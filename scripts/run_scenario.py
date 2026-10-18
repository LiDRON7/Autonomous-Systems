#!/usr/bin/env python3
"""Own one simulation lifecycle; no ROS or third-party dependencies on the host."""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

ROOT = Path(__file__).resolve().parents[1]


def load_scenario(path):
    config = json.loads(Path(path).read_text())
    if not isinstance(config, dict):
        raise ValueError('scenario must be an object')
    required = {'name', 'version', 'environment', 'uav', 'mission', 'timeouts', 'expected'}
    if set(config) != required:
        raise ValueError('scenario fields must be: ' + ', '.join(sorted(required)))
    if not isinstance(config['name'], str) or not config['name'].strip():
        raise ValueError('name must be a nonempty string')
    if type(config['version']) is not int or config['version'] != 1:
        raise ValueError('only scenario version 1 is supported')
    # V1 intentionally uses the Simulation repository's existing default world/spawn.
    if config['environment'] != {'profile': 'default', 'obstacles': []}:
        raise ValueError('only default environment with no added obstacles is supported')
    if config['uav'] != {'start_mode': 'default'}:
        raise ValueError('only default UAV spawn is supported')
    mission = config['mission']
    if not isinstance(mission, dict) or set(mission) != {'frame', 'destination'}:
        raise ValueError('mission requires frame and destination')
    if mission['frame'] != 'local_ned':
        raise ValueError('only local_ned missions are supported')
    destination = mission['destination']
    if not isinstance(destination, dict) or set(destination) != {'north', 'east', 'down'}:
        raise ValueError('destination requires north, east, down')
    for value in destination.values():
        number(value)
    if not isinstance(config['timeouts'], dict) or set(config['timeouts']) != {'startup_s', 'mission_s', 'cleanup_s'}:
        raise ValueError('timeouts requires startup_s, mission_s, cleanup_s')
    for value in config['timeouts'].values():
        number(value, positive=True)
    expected = config['expected']
    if not isinstance(expected, dict) or set(expected) != {'states', 'forbidden_states', 'require_path', 'arrival_tolerance_m'}:
        raise ValueError('expected requires states, forbidden_states, require_path, arrival_tolerance_m')
    states = {'IDLE', 'PREFLIGHT', 'TAKEOFF', 'PLANNING', 'NAVIGATING', 'REPLANNING', 'APPROACH', 'LANDING_CHECK', 'LANDING', 'COMPLETE', 'HOLD', 'ABORT'}
    for key in ('states', 'forbidden_states'):
        values = expected[key]
        if not isinstance(values, list) or any(not isinstance(v, str) or v not in states for v in values):
            raise ValueError('expected states must be lists of known mission states')
    if set(expected['states']) & set(expected['forbidden_states']):
        raise ValueError('required and forbidden states overlap')
    if type(expected['require_path']) is not bool:
        raise ValueError('require_path must be boolean')
    number(expected['arrival_tolerance_m'], positive=True)
    return config


def number(value, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError('numeric values must be finite; deadlines and tolerances must be positive')


@contextmanager
def stack_lock():
    # One lock per user, independent of checkout, because container names are fixed.
    path = Path.home() / '.cache' / 'lidron-scenario.lock'
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another scenario runner owns the stack') from None
        yield


def stop_process(process):
    if process is not None and process.poll() is None:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def command(argv, timeout, **kwargs):
    process = subprocess.Popen(argv, start_new_session=True, **kwargs)
    try:
        return process.wait(timeout=max(0.01, timeout))
    finally:
        stop_process(process)


def execute(config, run_dir):
    result = {'execution_status': 'ERROR', 'reason': 'INFRASTRUCTURE_ERROR',
              'phase': 'startup', 'cleanup': 'not_needed', 'last_state': None}
    started = time.monotonic()
    launch = None
    owns_stack = False
    script = str(ROOT / 'scripts' / 'simulation.sh')
    startup_deadline = started + config['timeouts']['startup_s']
    try:
        # The lock protects runners; also refuse an already running manual stack.
        check = subprocess.run([script, 'running'], capture_output=True, text=True,
                               timeout=max(0.01, startup_deadline - time.monotonic()), check=True)
        if check.stdout.strip():
            raise RuntimeError('stack already running; stop it before running a scenario')
        owns_stack = True  # Cleanup is required even when launch only partly succeeds.
        with (run_dir / 'stack.log').open('w') as stack_log, (run_dir / 'mission.log').open('w') as mission_log:
            launch = subprocess.Popen([script, 'up'], stdout=stack_log, stderr=subprocess.STDOUT,
                                      start_new_session=True)
            while True:
                remaining = startup_deadline - time.monotonic()
                if remaining <= 0:
                    raise subprocess.TimeoutExpired('startup', config['timeouts']['startup_s'])
                if launch.poll() is not None:
                    raise RuntimeError('simulation launcher exited; inspect stack.log')
                # The container may not exist or have finished its first colcon build yet.
                relative = run_dir.relative_to(ROOT).as_posix()
                argv = ['docker', 'exec', 'lidron_autonomy', 'bash', '-c',
                        'source /autonomy/install/setup.bash && exec python3 /autonomy/scripts/scenario_ros.py "$@"',
                        'scenario_ros', f'/autonomy/{relative}', str(remaining)]
                code = command(argv, remaining + config['timeouts']['mission_s'] + 5,
                               stdout=mission_log, stderr=subprocess.STDOUT)
                worker_result = run_dir / 'execution.json'
                if worker_result.exists():
                    result.update(json.loads(worker_result.read_text()))
                    break
                if code not in (1, 75):
                    raise RuntimeError(f'ROS helper exited with code {code}; inspect mission.log')
                time.sleep(min(1, max(0, startup_deadline - time.monotonic())))
    except KeyboardInterrupt:
        result.update(execution_status='CANCELLED', reason='CANCELLED', message='execution interrupted')
    except subprocess.TimeoutExpired as error:
        result.update(execution_status='ERROR', reason='TIMEOUT', message=str(error))
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        result.update(execution_status='ERROR', reason='INFRASTRUCTURE_ERROR', message=str(error))
    finally:
        if owns_stack:
            # Stop the foreground compose/build process before teardown so it cannot restart services.
            stop_process(launch)
            result['cleanup'] = 'ok'
            try:
                with (run_dir / 'cleanup.log').open('w') as log:
                    code = command([script, 'down'], config['timeouts']['cleanup_s'],
                                   stdout=log, stderr=subprocess.STDOUT)
                if code:
                    raise RuntimeError(f'shutdown exited with code {code}')
            except (OSError, RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                result.update(original_status=result['execution_status'], original_reason=result['reason'],
                              execution_status='ERROR', reason='CLEANUP_ERROR', cleanup='failed',
                              cleanup_message=str(error))
        result['duration_s'] = round(time.monotonic() - started, 3)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('scenario', type=Path)
    args = parser.parse_args()
    # Invalid scenarios never launch or stop containers.
    try:
        config = load_scenario(args.scenario)
    except (OSError, ValueError) as error:
        print(json.dumps({'execution_status': 'ERROR', 'reason': 'CONFIG_INVALID', 'message': str(error)}))
        return 2
    run_id = time.strftime('%Y%m%dT%H%M%S', time.gmtime()) + '-' + uuid.uuid4().hex[:8]
    run_dir = ROOT / 'results' / run_id
    try:
        with stack_lock():
            run_dir.mkdir(parents=True)
            (run_dir / 'scenario.json').write_text(json.dumps(config, indent=2) + '\n')
            print(f'Running {config["name"]}; evidence: {run_dir}', flush=True)
            result = execute(config, run_dir)
            result.update(run_id=run_id, scenario=config['name'],
                          config_sha256=hashlib.sha256((run_dir / 'scenario.json').read_bytes()).hexdigest())
            pending = run_dir / 'result.json.tmp'
            pending.write_text(json.dumps(result, indent=2) + '\n')
            pending.replace(run_dir / 'result.json')
            print(json.dumps(result, indent=2))
    except (OSError, RuntimeError) as error:
        print(json.dumps({'execution_status': 'ERROR', 'reason': 'INFRASTRUCTURE_ERROR', 'message': str(error)}))
        return 2
    return {'COMPLETED': 0, 'FAILED': 1, 'TIMEOUT': 124, 'ERROR': 2, 'CANCELLED': 130}[result['execution_status']]


def interrupt(*_args):
    raise KeyboardInterrupt()


if __name__ == '__main__':
    signal.signal(signal.SIGTERM, interrupt)
    sys.exit(main())
