"""Lifecycle tests use fake processes; they never start Docker or a vehicle."""
import copy
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('runner', ROOT / 'scripts/run_scenario.py')
runner = importlib.util.module_from_spec(spec)
spec.loader.exec_module(runner)
EXAMPLE = json.loads((ROOT / 'scenarios/clear_path.json').read_text())


class ConfigTests(unittest.TestCase):
    def load(self, config):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'scenario.json'
            path.write_text(json.dumps(config))
            return runner.load_scenario(path)

    def test_example(self):
        self.assertEqual(self.load(EXAMPLE), EXAMPLE)

    def test_invalid_configurations(self):
        changes = [
            ('version', True), ('environment', {'profile': 'custom', 'obstacles': []}),
            ('uav', {'start_mode': 'custom'}), ('mission', {'frame': 'enu', 'destination': {}}),
            ('timeouts', {'startup_s': 0, 'mission_s': 1, 'cleanup_s': 1}),
            ('timeouts', {'startup_s': float('nan'), 'mission_s': 1, 'cleanup_s': 1}),
        ]
        for key, value in changes:
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.load({**EXAMPLE, key: value})
        for value in (float('inf'), True, '8'):
            config = copy.deepcopy(EXAMPLE)
            config['mission']['destination']['north'] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.load(config)
        config = copy.deepcopy(EXAMPLE)
        config['expected']['forbidden_states'].append('COMPLETE')
        with self.assertRaises(ValueError):
            self.load(config)


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name)
        (self.root / 'scripts').mkdir()
        self.run_dir = self.root / 'results/test'
        self.run_dir.mkdir(parents=True)
        (self.run_dir / 'scenario.json').write_text(json.dumps(EXAMPLE))
        script = self.root / 'scripts/simulation.sh'
        script.write_text('''#!/bin/sh
case "$1" in
running) [ "$FAKE_MODE" != active ] || echo existing-container ;;
up) touch "$FAKE_ROOT/launched"; [ "$FAKE_MODE" != launch_error ] || exit 1; exec sleep 60 ;;
down) touch "$FAKE_ROOT/cleaned"; [ "$FAKE_MODE" != cleanup_error ] ;;
esac
''')
        script.chmod(0o755)
        docker = self.root / 'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os
from pathlib import Path
root = Path(os.environ['FAKE_ROOT'])
mode = os.environ['FAKE_MODE']
if mode == 'not_ready':
    raise SystemExit(75)
status = {'failed': 'FAILED', 'timeout': 'TIMEOUT'}.get(mode, 'COMPLETED')
reason = {'failed': 'MISSION_FAILURE', 'timeout': 'TIMEOUT'}.get(mode, 'NONE')
(root / 'results/test/execution.json').write_text(json.dumps({
    'execution_status': status, 'reason': reason, 'phase': 'mission', 'last_state': 'COMPLETE'}))
''')
        docker.chmod(0o755)
        self.config = copy.deepcopy(EXAMPLE)
        self.config['timeouts'] = {'startup_s': 3, 'mission_s': 1, 'cleanup_s': 3}
        self.addCleanup(patch.stopall)
        patch.object(runner, 'ROOT', self.root).start()
        patch.dict(os.environ, {'PATH': str(self.root) + os.pathsep + os.environ['PATH'],
                               'FAKE_ROOT': str(self.root), 'FAKE_MODE': 'completed'}).start()

    def execute(self, mode):
        os.environ['FAKE_MODE'] = mode
        return runner.execute(self.config, self.run_dir)

    def test_terminal_results_and_cleanup(self):
        for mode, status in [('completed', 'COMPLETED'), ('failed', 'FAILED'), ('timeout', 'TIMEOUT')]:
            with self.subTest(mode=mode):
                result = self.execute(mode)
                self.assertEqual(result['execution_status'], status)
                self.assertEqual(result['cleanup'], 'ok')
                self.assertTrue((self.root / 'cleaned').exists())

    def test_startup_timeout(self):
        self.config['timeouts']['startup_s'] = 0.5
        result = self.execute('not_ready')
        self.assertEqual((result['execution_status'], result['reason'], result['phase']),
                         ('ERROR', 'TIMEOUT', 'startup'))
        self.assertEqual(result['cleanup'], 'ok')

    def test_cleanup_failure_preserves_outcome(self):
        result = self.execute('cleanup_error')
        self.assertEqual(result['reason'], 'CLEANUP_ERROR')
        self.assertEqual(result['original_status'], 'COMPLETED')
        self.assertEqual(result['cleanup'], 'failed')

    def test_existing_stack_is_not_stopped(self):
        result = self.execute('active')
        self.assertEqual(result['execution_status'], 'ERROR')
        self.assertFalse((self.root / 'cleaned').exists())
        self.assertFalse((self.root / 'launched').exists())

    def test_launch_failure_cleans_up(self):
        # Ensure launch has exited before the runner's next poll.
        real_command = runner.command
        def wait_for_launch(argv, *args, **kwargs):
            if argv[0] == 'docker':
                __import__('time').sleep(0.1)
                return 75
            return real_command(argv, *args, **kwargs)
        with patch.object(runner, 'command', side_effect=wait_for_launch):
            result = self.execute('launch_error')
        self.assertEqual(result['execution_status'], 'ERROR')
        self.assertEqual(result['cleanup'], 'ok')

    def test_cancellation_cleans_up(self):
        real_command = runner.command
        def cancel_worker(argv, *args, **kwargs):
            if argv[0] == 'docker':
                raise KeyboardInterrupt()
            return real_command(argv, *args, **kwargs)
        with patch.object(runner, 'command', side_effect=cancel_worker):
            result = self.execute('completed')
        self.assertEqual(result['execution_status'], 'CANCELLED')
        self.assertEqual(result['cleanup'], 'ok')

    def test_command_deadline_kills_process(self):
        with self.assertRaises(__import__('subprocess').TimeoutExpired):
            runner.command(['sleep', '60'], 0.05)


if __name__ == '__main__':
    unittest.main()
