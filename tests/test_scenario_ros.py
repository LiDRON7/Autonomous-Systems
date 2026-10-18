"""Exercise ROS orchestration with service and clock doubles, without ROS installed."""
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
spec = importlib.util.spec_from_file_location('worker', ROOT / 'scripts/scenario_ros.py')
worker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(worker)


class RosTests(unittest.TestCase):
    def execute(self, terminal='COMPLETE', preflight_ok=True, reject=None):
        config = json.loads((ROOT / 'scenarios/clear_path.json').read_text())
        config['timeouts']['mission_s'] = 1
        now = [0.0]
        calls = []
        enabled = [False]
        callback = [None]
        class Request:
            def __init__(self, **kwargs):
                self.__dict__.update(kwargs)
        service = NS(Request=Request)
        class Client:
            def __init__(self, name):
                self.name = name
            def service_is_ready(self):
                return True
            def call_async(self, request):
                calls.append((self.name, request.__dict__))
                if self.name.endswith('enable'):
                    enabled[0] = True
                response = NS(success=preflight_ok if self.name.endswith('preflight_check') else reject != 'enable',
                              accepted=reject != 'destination', message='test response')
                return NS(done=lambda: True, result=lambda: response)
        class Node:
            def __init__(self, name):
                pass
            def create_subscription(self, cls, topic, cb, qos):
                callback[0] = cb
            def create_client(self, cls, name):
                return Client(name)
            def get_node_names(self):
                return ['perception', 'mission', 'diagnostics']
            def destroy_node(self):
                pass
        def spin_once(node, timeout_sec):
            now[0] += 0.1
            callback[0](NS(data=terminal if enabled[0] else 'IDLE'))
        modules = {
            'rclpy': NS(init=lambda: None, shutdown=lambda: None, spin_once=spin_once),
            'rclpy.node': NS(Node=Node), 'std_msgs': NS(), 'std_msgs.msg': NS(String=object),
            'std_srvs': NS(), 'std_srvs.srv': NS(SetBool=service, Trigger=service),
            'interfaces': NS(), 'interfaces.srv': NS(SetLocalDestination=service),
        }
        with tempfile.TemporaryDirectory() as tmp, patch.dict('sys.modules', modules), patch.object(worker.time, 'monotonic', side_effect=lambda: now[0]):
            path = Path(tmp)
            (path / 'scenario.json').write_text(json.dumps(config))
            self.assertEqual(worker.run(path, 1), 0)
            result = json.loads((path / 'execution.json').read_text())
            events = [json.loads(line) for line in (path / 'events.jsonl').read_text().splitlines()]
        return result, calls, events

    def test_readiness_destination_enable_completion_order(self):
        result, calls, events = self.execute()
        self.assertEqual(result['execution_status'], 'COMPLETED')
        self.assertEqual([name for name, _ in calls], ['/autonomy/preflight_check', '/autonomy/set_local_destination', '/autonomy/enable'])
        self.assertEqual(calls[1][1], {'north': 8.0, 'east': 0.0, 'down': -5.0})
        kinds = [event['event'] for event in events]
        self.assertLess(kinds.index('ready'), kinds.index('destination'))
        self.assertLess(kinds.index('destination'), kinds.index('enable'))

    def test_failure_signals(self):
        for state in ('ABORT', 'HOLD'):
            with self.subTest(state=state):
                self.assertEqual(self.execute(terminal=state)[0]['execution_status'], 'FAILED')

    def test_mission_timeout(self):
        result, _, _ = self.execute(terminal='NAVIGATING')
        self.assertEqual((result['execution_status'], result['phase']), ('TIMEOUT', 'mission'))

    def test_failed_preflight_never_starts_mission(self):
        result, calls, _ = self.execute(preflight_ok=False)
        self.assertEqual((result['execution_status'], result['reason'], result['phase']), ('ERROR', 'TIMEOUT', 'startup'))
        self.assertTrue(all(name.endswith('preflight_check') for name, _ in calls))

    def test_command_rejections(self):
        for reject in ('destination', 'enable'):
            with self.subTest(reject=reject):
                result, calls, _ = self.execute(reject=reject)
                self.assertEqual(result['execution_status'], 'ERROR')
                if reject == 'destination':
                    self.assertFalse(any(name.endswith('enable') for name, _ in calls))


if __name__ == '__main__':
    unittest.main()
