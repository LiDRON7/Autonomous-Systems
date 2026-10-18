"""No Docker/ROS needed: actual fake processes exercise lifecycle failure paths."""
import copy
import io
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from scenario_framework import runner
from scenario_framework.contracts import EventStream, ResultStore, exit_code, outcome
from scenario_framework.validation import Validator
ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = json.loads((ROOT / 'scenarios/clear_path.json').read_text())


class ContractTests(unittest.TestCase):
    def test_scenario_schema_and_semantic_rejections(self):
        configs = []
        for key, value in [('version', True), ('version', 2), ('environment', {'profile': 'custom', 'obstacles': []}),
                           ('uav', {'start_mode': 'custom'}), ('name', ' '), ('mission', {}), ('expected', [])]:
            configs.append({**EXAMPLE, key: value})
        for value in (float('inf'), float('nan'), True, '8'):
            c = copy.deepcopy(EXAMPLE); c['mission']['destination']['north'] = value; configs.append(c)
        for key in ('startup_s', 'mission_s', 'cleanup_s'):
            c = copy.deepcopy(EXAMPLE); c['timeouts'][key] = 0; configs.append(c)
        c = copy.deepcopy(EXAMPLE); c['expected']['forbidden_states'].append('COMPLETE'); configs.append(c)
        c = copy.deepcopy(EXAMPLE); c['triggers'] = []; configs.append(c)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / 'scenario.json'
            path.write_text(json.dumps(EXAMPLE)); self.assertEqual(runner.load_scenario(path), EXAMPLE)
            for config in configs:
                with self.subTest(config=config):
                    path.write_text(json.dumps(config))
                    with self.assertRaises(ValueError): runner.load_scenario(path)

    def test_events_resume_sequence_and_preserve_timestamps(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {'run_id': 'r', 'started_monotonic_s': 0}
            a = EventStream(tmp, manifest)
            a.emit('state', '/autonomy/state', {'previous': 'IDLE', 'current': 'PREFLIGHT'})
            a.close()
            b = EventStream(tmp, manifest)
            b.emit('process_exit', 'runner', {'process': 'test', 'exit_code': 0}, 12.3); b.close()
            events = [json.loads(line) for line in (Path(tmp)/'events.jsonl').read_text().splitlines()]
            self.assertEqual([e['seq'] for e in events], [1, 2])
            self.assertIsNone(events[0]['ros_time_s'])
            self.assertEqual(events[1]['ros_time_s'], 12.3)
            for e in events:
                self.assertEqual(set(e), {'schema_version','run_id','seq','elapsed_s','ros_time_s','source','kind','payload'})

    def test_two_writers_do_not_reuse_sequence_numbers(self):
        with tempfile.TemporaryDirectory() as tmp:
            manifest = {'run_id': 'r', 'started_monotonic_s': 0}
            a = EventStream(tmp, manifest); b = EventStream(tmp, manifest)
            for writer in (a, b, a):
                writer.emit('process_exit', 'runner', {'process': 'test', 'exit_code': 0})
            a.close(); b.close()
            values = [json.loads(line)['seq'] for line in (Path(tmp)/'events.jsonl').read_text().splitlines()]
            self.assertEqual(values, [1, 2, 3])

    def test_validator_never_infers_pass_from_complete(self):
        result = Validator.evaluate(EXAMPLE, [], 'COMPLETED')
        self.assertEqual((result['verdict'], result['execution_status'], result['reason']), ('FAIL','ERROR','MONITOR_DATA_MISSING'))

    def test_observed_forbidden_state_is_expectation_failure(self):
        result = Validator.evaluate(EXAMPLE, [{'kind':'state','seq':7,'payload':{'current':'HOLD'}}], 'COMPLETED')
        self.assertEqual((result['verdict'], result['execution_status'], result['reason']), ('FAIL','COMPLETED','EXPECTATION_FAILED'))
        self.assertEqual(result['expectation_checks'][0]['evidence_seq'], [7])

    def test_exit_codes(self):
        for status, verdict, code in [('COMPLETED','PASS',0), ('COMPLETED','FAIL',1),
                                      ('TIMED_OUT','FAIL',124), ('ERROR','FAIL',2), ('CANCELLED','FAIL',130)]:
            self.assertEqual(exit_code({'execution_status':status,'verdict':verdict}), code)

    def test_finalization_failure_preserves_provisional(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp); (path/'provisional-result.json').write_text('{}')
            with patch('scenario_framework.contracts.atomic_json', side_effect=OSError('disk full')):
                with self.assertRaises(OSError): ResultStore.finalize(path, outcome())
            self.assertTrue((path/'provisional-result.json').exists())


class LifecycleTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.root = Path(self.tmp.name); (self.root/'scripts').mkdir()
        self.directory = self.root/'results/test'; self.directory.mkdir(parents=True)
        (self.directory/'scenario.json').write_text(json.dumps(EXAMPLE))
        script = self.root/'scripts/simulation.sh'
        script.write_text('''#!/bin/sh
case "$1" in
running) [ "$FAKE_MODE" != active ] || echo existing-container ;;
up) touch "$FAKE_ROOT/launched"; [ "$FAKE_MODE" != launch_error ] || exit 1; exec sleep 60 ;;
down) touch "$FAKE_ROOT/cleaned"; [ "$FAKE_MODE" != cleanup_error ] ;;
esac
'''); script.chmod(0o755)
        docker = self.root/'docker'
        docker.write_text('''#!/usr/bin/env python3
import json, os, time
from pathlib import Path
r=Path(os.environ['FAKE_ROOT'])/'results/test'
mode=os.environ['FAKE_MODE']
if mode=='not_ready': raise SystemExit(75)
if mode=='hung': time.sleep(60)
if mode=='hung_mission':
 (r/'progress.json').write_text(json.dumps({'phase':'mission','deadline':time.monotonic()+0.1}))
 time.sleep(60)
status='TIMED_OUT' if mode=='timeout' else 'COMPLETED'
reason='TIMEOUT' if mode=='timeout' else 'NONE'
(r/'execution.json').write_text(json.dumps({'execution_status':status,'reason':reason,'phase':'mission',
 'verdict':'FAIL','reasons':[] if reason=='NONE' else [{'code':reason,'phase':'mission'}]}))
state='HOLD' if mode in ('failed','cleanup_error') else 'COMPLETE'
(r/'events.jsonl').write_text(json.dumps({'schema_version':1,'run_id':'test','seq':1,'elapsed_s':0.1,
 'ros_time_s':None,'source':'/autonomy/state','kind':'state','payload':{'previous':'IDLE','current':state}})+'\\n')
'''); docker.chmod(0o755)
        self.config = copy.deepcopy(EXAMPLE); self.config['timeouts']={'startup_s':3,'mission_s':1,'cleanup_s':3}
        self.addCleanup(patch.stopall)
        patch.object(runner,'ROOT',self.root).start()
        patch.object(runner,'ownership_directory',return_value=self.root).start()
        patch.object(runner,'source_commit',return_value='abc123').start()
        patch.dict(os.environ,{'PATH':str(self.root)+os.pathsep+os.environ['PATH'],'FAKE_ROOT':str(self.root),'FAKE_MODE':'completed'}).start()

    def execute(self, mode):
        os.environ['FAKE_MODE']=mode
        return runner.execute(self.config,self.directory)

    def test_completed_is_unverified_until_validator_exists(self):
        r=self.execute('completed')
        self.assertEqual((r['execution_status'],r['reason'],r['cleanup_status']),('ERROR','MONITOR_DATA_MISSING','COMPLETED'))
        from jsonschema import validate
        validate(r, json.loads((ROOT/'scenario_framework/result.schema.json').read_text()))
        for line in (self.directory/'events.jsonl').read_text().splitlines():
            validate(json.loads(line), json.loads((ROOT/'scenario_framework/event.schema.json').read_text()))
        self.assertTrue((self.directory/'provisional-result.json').exists())
        self.assertFalse((self.root/'dirty.json').exists())
        for key in ('schema_version','run_id','scenario_name','verdict','execution_status','reason','phase',
                    'duration_s','cleanup_status','original_outcome','reasons','start_utc','source_commits',
                    'configuration_sha256','expectation_checks','artifact_paths'):
            self.assertIn(key,r)

    def test_observed_failure_and_timeout(self):
        for mode,status in [('failed','COMPLETED'),('timeout','TIMED_OUT')]:
            with self.subTest(mode=mode):
                r=self.execute(mode); self.assertEqual(r['execution_status'],status);self.assertEqual(r['verdict'],'FAIL')
                self.assertEqual(r['cleanup_status'],'COMPLETED')

    def test_startup_timeout_and_hung_worker(self):
        for mode in ('not_ready','hung'):
            with self.subTest(mode=mode):
                self.config['timeouts']['startup_s']=1.5
                r=self.execute(mode)
                self.assertEqual((r['execution_status'],r['reason'],r['phase']),('ERROR','TIMEOUT','startup'))
                self.assertEqual(r['cleanup_status'],'COMPLETED')

    def test_hung_mission_deadline(self):
        r=self.execute('hung_mission')
        self.assertEqual((r['execution_status'],r['reason'],r['phase']),('TIMED_OUT','TIMEOUT','mission'))

    def test_cleanup_error_preserves_verdict_and_prior_reasons_and_latches(self):
        r=self.execute('cleanup_error')
        self.assertEqual((r['execution_status'],r['reason'],r['phase'],r['cleanup_status']),('ERROR','CLEANUP_ERROR','cleanup','ERROR'))
        self.assertEqual(r['original_outcome'], {'execution_status':'COMPLETED','verdict':'FAIL','reason':'EXPECTATION_FAILED','phase':'validation'})
        self.assertEqual([v['code'] for v in r['reasons']], ['EXPECTATION_FAILED','CLEANUP_ERROR'])
        self.assertTrue((self.root/'dirty.json').exists())
        r=self.execute('completed'); self.assertIn('previous cleanup failed',r['message'])
        runner.reset(3);self.assertFalse((self.root/'dirty.json').exists())

    def test_existing_stack_is_untouched(self):
        r=self.execute('active');self.assertEqual(r['execution_status'],'ERROR')
        self.assertFalse((self.root/'cleaned').exists());self.assertFalse((self.root/'launched').exists())

    def test_partial_launch_failure_cleans_up(self):
        r=self.execute('launch_error');self.assertEqual(r['execution_status'],'ERROR')
        self.assertEqual(r['cleanup_status'],'COMPLETED')

    def test_cancellation_cleans_up(self):
        with patch.object(runner,'wait_worker',side_effect=KeyboardInterrupt):r=self.execute('completed')
        self.assertEqual(r['execution_status'],'CANCELLED');self.assertEqual(r['cleanup_status'],'COMPLETED')

    def test_exclusive_lock(self):
        with runner.stack_lock():
            with self.assertRaises(RuntimeError):
                with runner.stack_lock(): pass

    def test_cli_unwritable_output_reports_infrastructure_error(self):
        scenario=self.root/'scenario.json';scenario.write_text(json.dumps(EXAMPLE))
        with patch.object(Path,'mkdir',side_effect=PermissionError('read-only')),patch('sys.stderr',new_callable=io.StringIO) as err:
            self.assertEqual(runner.main(['run',str(scenario),'--output',str(self.root/'results')]),2)
        self.assertEqual(json.loads(err.getvalue())['reason'],'INFRASTRUCTURE_ERROR')

    def test_invalid_scenario_never_launches(self):
        scenario=self.root/'bad.json';scenario.write_text('{}')
        with patch('sys.stderr',new_callable=io.StringIO) as err:
            self.assertEqual(runner.main(['run',str(scenario)]),2)
        self.assertEqual(json.loads(err.getvalue())['phase'],'config')
        self.assertFalse((self.root/'launched').exists())


if __name__=='__main__':unittest.main()
