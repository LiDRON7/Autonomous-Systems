"""Simulation ownership, bounded execution, and Appendix A result finalization."""
import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from .config import load_scenario
from .contracts import EventStream, ResultStore, atomic_json, exit_code, outcome, utc_now
from .validation import Validator

ROOT = Path(__file__).resolve().parents[1]


def ownership_directory():
    directory = Path.home() / '.cache' / 'lidron-scenarios'
    directory.mkdir(parents=True, exist_ok=True)
    return directory


@contextmanager
def stack_lock():
    with (ownership_directory() / 'stack.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise RuntimeError('another scenario runner owns the stack') from None
        yield


def stop_process(process):
    if process is not None and process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
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


def source_commit(directory):
    try:
        value = subprocess.run(['git', '-C', str(directory), 'rev-parse', 'HEAD'],
                               capture_output=True, text=True, timeout=2, check=True)
        return value.stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return None


def make_manifest(config, run_id):
    simulation = Path(os.environ.get('SIMULATION_ROOT', ROOT.parent / 'Simulation'))
    return {'schema_version': 1, 'run_id': run_id, 'scenario_name': config.get('name'),
            'start_utc': utc_now(), 'started_monotonic_s': time.monotonic(),
            'source_commits': {'autonomous_systems': source_commit(ROOT),
                               'simulation': source_commit(simulation)},
            'configuration_sha256': hashlib.sha256(json.dumps(config, sort_keys=True,
                                            allow_nan=False).encode()).hexdigest()}


def wait_worker(argv, directory, manifest, config, launch, log, exits):
    worker = subprocess.Popen(argv, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        while worker.poll() is None:
            progress_path = directory / 'progress.json'
            progress = json.loads(progress_path.read_text()) if progress_path.exists() else None
            phase = progress['phase'] if progress else 'startup'
            deadline = progress['deadline'] if progress else manifest['started_monotonic_s'] + config['timeouts']['startup_s']
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(phase, config['timeouts'][phase + '_s'])
            if launch.poll() is not None:
                raise RuntimeError('simulation launcher exited during execution')
            time.sleep(0.05)
        return worker.returncode
    finally:
        stop_process(worker)
        exits.append({'process': 'scenario_ros', 'exit_code': worker.returncode})


def execute(config, run_dir, manifest=None):
    manifest = manifest or make_manifest(config, run_dir.name)
    atomic_json(run_dir / 'manifest.json', manifest)
    result = outcome()
    result['last_state'] = None
    result['cleanup_status'] = 'NOT_REQUIRED'
    launch = None
    owns_stack = False
    dirty = ownership_directory() / 'dirty.json'
    script = str(ROOT / 'scripts/simulation.sh')
    startup_deadline = manifest['started_monotonic_s'] + config['timeouts']['startup_s']
    exits = []
    try:
        if dirty.exists():
            raise RuntimeError('previous cleanup failed; run python -m scenario_framework reset first')
        if any(value is None for value in manifest['source_commits'].values()):
            raise RuntimeError('both repository commit IDs are required; verify SIMULATION_ROOT')
        check = subprocess.run([script, 'running'], capture_output=True, text=True,
                               timeout=max(0.01, startup_deadline-time.monotonic()), check=True)
        if check.stdout.strip():
            raise RuntimeError('stack already running; stop it before running a scenario')
        owns_stack = True
        # Mark before launch: an abruptly killed runner must not silently allow reuse.
        atomic_json(dirty, {'run_id': manifest['run_id'], 'directory': str(run_dir)})
        with (run_dir / 'stack.log').open('w') as log, (run_dir / 'mission.log').open('w') as mission_log:
            launch = subprocess.Popen([script, 'up'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            while True:
                if time.monotonic() >= startup_deadline:
                    raise subprocess.TimeoutExpired('startup', config['timeouts']['startup_s'])
                if launch.poll() is not None:
                    raise RuntimeError('simulation launcher exited; inspect stack.log')
                relative = run_dir.relative_to(ROOT).as_posix()
                argv = ['docker', 'exec', 'lidron_autonomy', 'bash', '-c',
                        'source /autonomy/install/setup.bash && cd /autonomy && exec python3 -m scenario_framework.ros "$@"',
                        'scenario_ros', '/autonomy/' + relative]
                code = wait_worker(argv, run_dir, manifest, config, launch, mission_log, exits)
                worker_result = run_dir / 'execution.json'
                if worker_result.exists():
                    result.update(json.loads(worker_result.read_text()))
                    if result['execution_status'] == 'COMPLETED':
                        with (run_dir / 'events.jsonl').open() as stream:
                            validation = Validator.evaluate(config, [json.loads(line) for line in stream], 'COMPLETED')
                        result.update(validation)
                    break
                if (run_dir / 'progress.json').exists() or code not in (1, 75):
                    raise RuntimeError(f'ROS helper exited with code {code}; inspect mission.log')
                time.sleep(min(1, max(0, startup_deadline-time.monotonic())))
    except KeyboardInterrupt:
        result.update(outcome('CANCELLED', 'CANCELLED', current_phase(run_dir), 'execution interrupted'))
    except subprocess.TimeoutExpired as error:
        phase = error.cmd if error.cmd in ('startup', 'mission') else 'startup'
        result.update(outcome('TIMED_OUT' if phase == 'mission' else 'ERROR', 'TIMEOUT', phase))
    except (OSError, ValueError, RuntimeError, subprocess.CalledProcessError) as error:
        result.update(outcome('ERROR', 'INFRASTRUCTURE_ERROR', current_phase(run_dir), str(error)))
    finally:
        # Persist a provisional outcome before cleanup; retain it if final writing fails.
        try:
            atomic_json(run_dir / 'provisional-result.json', {**manifest, **result})
        finally:
            if owns_stack:
                try:
                    stop_process(launch)
                    exits.append({'process': 'simulation.sh up', 'exit_code': launch.returncode if launch else None})
                    with (run_dir / 'cleanup.log').open('w') as log:
                        code = command([script, 'down'], config['timeouts']['cleanup_s'], stdout=log, stderr=subprocess.STDOUT)
                    exits.append({'process': 'simulation.sh down', 'exit_code': code})
                    if code:
                        raise RuntimeError(f'shutdown exited with code {code}')
                    result['cleanup_status'] = 'COMPLETED'
                    dirty.unlink(missing_ok=True)
                except (OSError, RuntimeError, subprocess.TimeoutExpired, KeyboardInterrupt) as error:
                    original = {key: result[key] for key in ('execution_status', 'verdict', 'reason', 'phase')}
                    reasons = list(result['reasons']) + [{'code': 'CLEANUP_ERROR', 'phase': 'cleanup'}]
                    result.update(outcome('ERROR', 'CLEANUP_ERROR', 'cleanup', str(error)))
                    result.update(original_outcome=original, reasons=reasons, cleanup_status='ERROR')
            # A failed teardown may leave a remote helper alive; EventStream serializes writers.
            stream = EventStream(run_dir, manifest)
            try:
                for value in exits:
                    if value['exit_code'] is not None:
                        stream.emit('process_exit', 'runner', value)
            finally:
                stream.close()
    result.update({key: value for key, value in manifest.items() if key != 'started_monotonic_s'})
    result['duration_s'] = round(time.monotonic()-manifest['started_monotonic_s'], 3)
    result['artifact_paths'] = {p.name: p.name for p in sorted(run_dir.iterdir()) if p.is_file() and not p.name.endswith('.tmp')}
    result['artifact_paths']['result.json'] = 'result.json'
    return result


class Runner:
    @staticmethod
    def run(config, directory, manifest=None):
        return execute(config, directory, manifest)


def current_phase(directory):
    try:
        return json.loads((directory / 'progress.json').read_text())['phase']
    except (OSError, ValueError, KeyError):
        return 'startup'


def reset(timeout):
    """Explicit full teardown is the only way to clear a dirty-stack latch."""
    with stack_lock():
        code = command([str(ROOT / 'scripts/simulation.sh'), 'down'], timeout)
        if code:
            raise RuntimeError('reset teardown failed; dirty latch retained')
        check = subprocess.run([str(ROOT / 'scripts/simulation.sh'), 'running'], capture_output=True,
                               text=True, check=True, timeout=timeout)
        if check.stdout.strip():
            raise RuntimeError('reset left running containers; dirty latch retained')
        (ownership_directory() / 'dirty.json').unlink(missing_ok=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    run_parser = sub.add_parser('run')
    run_parser.add_argument('scenario', type=Path)
    run_parser.add_argument('--output', type=Path, default=ROOT / 'results')
    reset_parser = sub.add_parser('reset')
    reset_parser.add_argument('--cleanup-timeout', type=float, default=30)
    args = parser.parse_args(argv)
    result = {**outcome(), 'schema_version': 1, 'run_id': uuid.uuid4().hex,
              'scenario_name': None, 'start_utc': utc_now(), 'duration_s': 0.0,
              'source_commits': {'autonomous_systems': source_commit(ROOT),
                                 'simulation': source_commit(Path(os.environ.get('SIMULATION_ROOT', ROOT.parent / 'Simulation')))},
              'configuration_sha256': None, 'cleanup_status': 'NOT_REQUIRED', 'artifact_paths': {}}
    directory = None
    try:
        if args.action == 'reset':
            if not 0 < args.cleanup_timeout < float('inf'):
                raise ValueError('cleanup timeout must be finite and positive')
            reset(args.cleanup_timeout)
            print('Full teardown verified; stack may be launched again.')
            return 0
        try:
            config = load_scenario(args.scenario)
        except (ValueError, OSError) as error:
            result.update(outcome('ERROR', 'CONFIG_INVALID', 'config', str(error)))
            print(json.dumps(result), file=sys.stderr)
            return 2
        output = args.output.resolve()
        if not output.is_relative_to(ROOT.resolve()):
            raise ValueError('--output must be inside the repository mounted at /autonomy')
        run_id = time.strftime('%Y%m%dT%H%M%S', time.gmtime()) + '-' + uuid.uuid4().hex[:8]
        manifest = make_manifest(config, run_id)
        directory = output / run_id
        with stack_lock():
            directory.mkdir(parents=True)
            atomic_json(directory / 'scenario.json', config)
            print(f'Running {config["name"]}; evidence: {directory}', flush=True)
            result = Runner.run(config, directory, manifest)
            ResultStore.finalize(directory, result)
            print(json.dumps(result, indent=2))
        return exit_code(result)
    except (OSError, ValueError, RuntimeError, ImportError, subprocess.SubprocessError) as error:
        # Provisional evidence is intentionally untouched, including on finalization failure.
        result.update(outcome('ERROR', 'INFRASTRUCTURE_ERROR', 'cleanup' if directory else 'startup', str(error)))
        print(json.dumps(result), file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        result.update(outcome('CANCELLED', 'CANCELLED'))
        print(json.dumps(result), file=sys.stderr)
        return 130


def interrupt(*_args):
    raise KeyboardInterrupt()
