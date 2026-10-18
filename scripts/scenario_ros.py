#!/usr/bin/env python3
"""ROS adapter run inside lidron_autonomy, after the workspace is sourced."""
import json
from pathlib import Path
import sys
import time


def run(run_dir, startup_s):
    # Imports can fail while the first workspace build is still in progress.
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        from interfaces.srv import SetLocalDestination
    except ImportError:
        return 75

    config = json.loads((run_dir / 'scenario.json').read_text())
    result = {'execution_status': 'ERROR', 'reason': 'INFRASTRUCTURE_ERROR',
              'phase': 'startup', 'last_state': None}
    rclpy.init()
    node = Node('scenario_executor')
    deadline = time.monotonic() + startup_s
    events = (run_dir / 'events.jsonl').open('a')
    started = time.monotonic()
    mission_started = False
    state = None
    state_stamp = None

    def event(kind, **values):
        events.write(json.dumps({'elapsed_s': time.monotonic() - started, 'event': kind, **values}) + '\n')
        events.flush()

    def receive(msg):
        nonlocal state, state_stamp
        state_stamp = time.monotonic()
        if state != msg.data:
            state = msg.data
            event('state', state=state)

    # Observe before submitting a destination or enabling the mission.
    subscription = node.create_subscription(String, '/autonomy/state', receive, 10)
    preflight = node.create_client(Trigger, '/autonomy/preflight_check')
    destination = node.create_client(SetLocalDestination, '/autonomy/set_local_destination')
    enable = node.create_client(SetBool, '/autonomy/enable')

    def spin():
        if time.monotonic() >= deadline:
            raise TimeoutError('wall-clock deadline expired')
        rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline - time.monotonic())))

    def call(client, request):
        future = client.call_async(request)
        while not future.done():
            spin()
        response = future.result()
        if response is None:
            raise RuntimeError('service returned no response')
        return response

    try:
        event('waiting_for_readiness')
        while True:
            spin()
            nodes = set(node.get_node_names())
            if not {'perception', 'mission', 'diagnostics'} <= nodes:
                continue
            if not all(c.service_is_ready() for c in (preflight, destination, enable)):
                continue
            if state != 'IDLE' or state_stamp is None or time.monotonic() - state_stamp > 2:
                continue
            response = call(preflight, Trigger.Request())
            event('preflight', success=response.success, message=response.message)
            if response.success:
                break
            # Avoid a busy loop while sensors are starting.
            retry_at = time.monotonic() + 1
            while time.monotonic() < retry_at:
                spin()
        event('ready')
        request = SetLocalDestination.Request(**config['mission']['destination'])
        response = call(destination, request)
        event('destination', accepted=response.accepted, message=response.message)
        if not response.accepted:
            raise RuntimeError('destination rejected: ' + response.message)
        # Start the mission deadline before enabling, so a stuck service is bounded too.
        result['phase'] = 'mission'
        deadline = time.monotonic() + config['timeouts']['mission_s']
        response = call(enable, SetBool.Request(data=True))
        event('enable', success=response.success, message=response.message)
        if not response.success:
            raise RuntimeError('mission enable rejected: ' + response.message)
        mission_started = True
        while state not in {'COMPLETE', 'ABORT', 'HOLD'}:
            spin()
        result.update(execution_status='COMPLETED' if state == 'COMPLETE' else 'FAILED',
                      reason='NONE' if state == 'COMPLETE' else 'MISSION_FAILURE')
    except TimeoutError as error:
        result.update(execution_status='TIMEOUT' if result['phase'] == 'mission' else 'ERROR',
                      reason='TIMEOUT', message=str(error))
    except KeyboardInterrupt:
        result.update(execution_status='CANCELLED', reason='CANCELLED')
    except Exception as error:
        result.update(execution_status='ERROR', reason='INFRASTRUCTURE_ERROR', message=str(error))
    finally:
        result['last_state'] = state
        result['mission_started'] = mission_started
        event('execution_finished', **result)
        pending = run_dir / 'execution.json.tmp'
        pending.write_text(json.dumps(result, indent=2) + '\n')
        pending.replace(run_dir / 'execution.json')
        events.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(run(Path(sys.argv[1]), float(sys.argv[2])))
