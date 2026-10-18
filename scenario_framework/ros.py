"""Simulation ROS adapter and contract monitor; no validation PASS is inferred."""
import json
import math
from pathlib import Path
import sys
import time
import uuid

from .contracts import EventStream, atomic_json, outcome


class MonitorDataMissing(RuntimeError):
    pass


class Monitor:
    def __init__(self, node, directory, manifest):
        from std_msgs.msg import Bool, Float32, String
        from nav_msgs.msg import Path as RosPath
        from px4_msgs.msg import VehicleLocalPosition
        from rclpy.qos import qos_profile_sensor_data
        self.node = node
        self.events = EventStream(directory, manifest)
        self.state = None
        self.received = {}
        self.valid_position = False
        self.subscriptions = []
        for cls, topic, callback, qos in [
            (String, '/autonomy/state', self.state_message, 10),
            (RosPath, '/autonomy/path', self.path_message, 10),
            (VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.position_message, qos_profile_sensor_data),
            (Bool, '/perception/obstacle_detected', self.obstacle_message, 10),
            (Float32, '/perception/obstacle_distance', self.distance_message, 10),
        ]:
            self.subscriptions.append(node.create_subscription(cls, topic, callback, qos))

    def start(self, run_id):
        if run_id != self.events.manifest['run_id']:
            raise ValueError('monitor run ID mismatch')
        # Subscriptions are created synchronously; discovery additionally verifies publishers.
        topics = ('/autonomy/state', '/autonomy/path', '/fmu/out/vehicle_local_position',
                  '/perception/obstacle_detected')
        return all(self.node.count_publishers(topic) > 0 for topic in topics)

    def touch(self, kind):
        self.received[kind] = time.monotonic()

    def fresh(self):
        return self.valid_position and all(time.monotonic()-self.received.get(k, -float('inf')) <= 2
                                           for k in ('state', 'position', 'obstacle'))

    def state_message(self, msg):
        self.touch('state')
        if msg.data != self.state:
            self.events.emit('state', '/autonomy/state', {'previous': self.state or '', 'current': msg.data})
            self.state = msg.data

    def path_message(self, msg):
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        points = [[p.pose.position.x, p.pose.position.y, p.pose.position.z] for p in msg.poses]
        if any(not math.isfinite(v) for point in points for v in point):
            raise MonitorDataMissing('nonfinite path point')
        self.events.emit('path', '/autonomy/path', {'path_id': f'{self.events.manifest["run_id"]}:{uuid.uuid4().hex}',
                         'pose_count': len(points), 'frame_id': msg.header.frame_id, 'points': points}, stamp)

    def position_message(self, msg):
        self.touch('position')
        finite = all(math.isfinite(v) for v in (msg.x, msg.y, msg.z))
        self.valid_position = bool(msg.xy_valid and msg.z_valid and finite)
        # Invalid numeric samples remain evidence without emitting forbidden JSON NaN/Infinity.
        if not finite:
            raise MonitorDataMissing('nonfinite PX4 position')
        self.events.emit('position', '/fmu/out/vehicle_local_position',
                         {'north': float(msg.x), 'east': float(msg.y), 'down': float(msg.z),
                          'frame_id': 'px4_local_ned', 'valid': self.valid_position,
                          'source_stamp_s': msg.timestamp / 1e6, 'source_clock': 'px4_boot'})

    def obstacle_message(self, msg):
        self.touch('obstacle')
        payload = {'detected': bool(msg.data)}
        if getattr(self, 'distance', None) is not None:
            payload['distance_m'] = self.distance
        self.events.emit('obstacle', '/perception/obstacle_detected', payload)

    def distance_message(self, msg):
        # This message has no detected flag: retain the latest value for the next Bool event.
        self.distance = float(msg.data) if math.isfinite(msg.data) else None

    def stop(self):
        self.events.close()


def run(directory):
    try:
        import rclpy
        from rclpy.node import Node
        from std_srvs.srv import SetBool, Trigger
        from interfaces.srv import SetLocalDestination
        from nav_msgs.msg import Path as RosPath
        from px4_msgs.msg import VehicleLocalPosition
    except ImportError:
        return 75  # Workspace may still be building.
    directory = Path(directory)
    config = json.loads((directory / 'scenario.json').read_text())
    manifest = json.loads((directory / 'manifest.json').read_text())
    result = outcome()
    deadline = manifest['started_monotonic_s'] + config['timeouts']['startup_s']
    rclpy.init()
    node = Node('scenario_executor')
    monitor = None
    mission_started = False
    try:
        monitor = Monitor(node, directory, manifest)
        preflight = node.create_client(Trigger, '/autonomy/preflight_check')
        destination = node.create_client(SetLocalDestination, '/autonomy/set_local_destination')
        enable = node.create_client(SetBool, '/autonomy/enable')

        def spin():
            if time.monotonic() >= deadline:
                raise TimeoutError('wall-clock deadline expired')
            rclpy.spin_once(node, timeout_sec=min(0.1, max(0, deadline-time.monotonic())))

        def call(client, request):
            future = client.call_async(request)
            while not future.done():
                spin()
            response = future.result()
            if response is None:
                raise RuntimeError('service returned no response')
            return response

        def service_event(service, response):
            payload = {'service': service, 'message': response.message}
            for name in ('accepted', 'success'):
                if hasattr(response, name):
                    payload[name] = bool(getattr(response, name))
            monitor.events.emit('service_response', service, payload)

        while True:
            spin()
            if not {'perception', 'mission', 'diagnostics'} <= set(node.get_node_names()):
                continue
            if not monitor.start(manifest['run_id']) or not monitor.fresh() or monitor.state != 'IDLE':
                continue
            if not all(c.service_is_ready() for c in (preflight, destination, enable)):
                continue
            response = call(preflight, Trigger.Request())
            service_event('/autonomy/preflight_check', response)
            if response.success:
                break
            retry_at = time.monotonic()+1
            while time.monotonic() < retry_at:
                spin()
        response = call(destination, SetLocalDestination.Request(**config['mission']['destination']))
        service_event('/autonomy/set_local_destination', response)
        if not response.accepted:
            raise RuntimeError('destination rejected: ' + response.message)
        deadline = time.monotonic()+config['timeouts']['mission_s']
        result['phase'] = 'mission'
        atomic_json(directory / 'progress.json', {'phase': 'mission', 'deadline': deadline})
        response = call(enable, SetBool.Request(data=True))
        service_event('/autonomy/enable', response)
        if not response.success:
            raise RuntimeError('mission enable rejected: ' + response.message)
        mission_started = True
        next_health = time.monotonic()+1
        while monitor.state not in {'COMPLETE', 'ABORT', 'HOLD'}:
            spin()
            if not monitor.fresh() or not monitor.start(manifest['run_id']):
                raise MonitorDataMissing('required monitor topics/valid position lost')
            if time.monotonic() >= next_health:
                response = call(preflight, Trigger.Request())
                service_event('/autonomy/preflight_check', response)
                if not response.success:
                    raise MonitorDataMissing('required sensor/PX4 telemetry lost: ' + response.message)
                next_health = time.monotonic()+1
        result.update(outcome('COMPLETED', 'NONE', 'mission'))
    except TimeoutError as error:
        result.update(outcome('TIMED_OUT' if result['phase'] == 'mission' else 'ERROR', 'TIMEOUT', result['phase'], str(error)))
    except MonitorDataMissing as error:
        result.update(outcome('ERROR', 'MONITOR_DATA_MISSING', result['phase'], str(error)))
    except KeyboardInterrupt:
        result.update(outcome('CANCELLED', 'CANCELLED', result['phase']))
    except Exception as error:
        result.update(outcome('ERROR', 'INFRASTRUCTURE_ERROR', result['phase'], str(error)))
    finally:
        result['last_state'] = monitor.state if monitor else None
        result['mission_started'] = mission_started
        try:
            atomic_json(directory / 'execution.json', result)
        finally:
            if monitor:
                monitor.stop()
            node.destroy_node()
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(run(Path(sys.argv[1])))
