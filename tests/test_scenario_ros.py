"""ROS doubles check subscription discovery, evidence payloads, and failure handling."""
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
from scenario_framework import ros
from scenario_framework.contracts import outcome

ROOT=Path(__file__).resolve().parents[1]


class RosTests(unittest.TestCase):
    def execute(self, terminal='COMPLETE', preflight_ok=True, reject=None, loss=False, no_path_publisher=False):
        config=json.loads((ROOT/'scenarios/clear_path.json').read_text())
        config['timeouts']={'startup_s':1,'mission_s':4,'cleanup_s':1}
        now=[0.0];calls=[];enabled=[False];callbacks={};position_messages=[True]
        class Request:
            def __init__(self,**kwargs):self.__dict__.update(kwargs)
        service=NS(Request=Request)
        class Client:
            def __init__(self,name):self.name=name
            def service_is_ready(self):return True
            def call_async(self,request):
                calls.append((self.name,request.__dict__))
                if self.name.endswith('enable'):enabled[0]=True
                response=NS(message='test response')
                if self.name.endswith('destination'):response.accepted=reject!='destination'
                else:response.success=preflight_ok if self.name.endswith('preflight_check') else reject!='enable'
                return NS(done=lambda:True,result=lambda:response)
        class Node:
            def __init__(self,name):pass
            def create_subscription(self,cls,topic,callback,qos):callbacks[topic]=callback;return object()
            def create_client(self,cls,name):return Client(name)
            def get_node_names(self):return ['perception','mission','diagnostics']
            def count_publishers(self,topic):return 0 if topic=='/autonomy/path' and no_path_publisher else 1
            def destroy_node(self):pass
        def spin_once(node,timeout_sec):
            now[0]+=0.1
            callbacks['/autonomy/state'](NS(data=terminal if enabled[0] else 'IDLE'))
            if not(enabled[0] and loss):
                callbacks['/fmu/out/vehicle_local_position'](NS(x=0.,y=0.,z=-5.,xy_valid=True,z_valid=True,timestamp=1000000))
                callbacks['/perception/obstacle_detected'](NS(data=False))
            # No path message before mission: discovery must suffice for readiness.
            if enabled[0]:
                callbacks['/autonomy/path'](NS(header=NS(frame_id='map',stamp=NS(sec=1,nanosec=2)),
                    poses=[NS(pose=NS(position=NS(x=1.,y=2.,z=-5.)))]))
        modules={'rclpy':NS(init=lambda:None,shutdown=lambda:None,spin_once=spin_once),
                 'rclpy.node':NS(Node=Node),'rclpy.qos':NS(qos_profile_sensor_data=object()),
                 'std_msgs':NS(),'std_msgs.msg':NS(String=object,Bool=object,Float32=object),
                 'std_srvs':NS(),'std_srvs.srv':NS(SetBool=service,Trigger=service),
                 'interfaces':NS(),'interfaces.srv':NS(SetLocalDestination=service),
                 'nav_msgs':NS(),'nav_msgs.msg':NS(Path=object),'px4_msgs':NS(),'px4_msgs.msg':NS(VehicleLocalPosition=object)}
        with tempfile.TemporaryDirectory() as tmp,patch.dict('sys.modules',modules),patch.object(ros.time,'monotonic',side_effect=lambda:now[0]):
            directory=Path(tmp)
            (directory/'scenario.json').write_text(json.dumps(config))
            (directory/'manifest.json').write_text(json.dumps({'run_id':'test','started_monotonic_s':0}))
            self.assertEqual(ros.run(directory),0)
            result=json.loads((directory/'execution.json').read_text())
            events=[json.loads(line) for line in (directory/'events.jsonl').read_text().splitlines()]
        from jsonschema import validate
        schema=json.loads((ROOT/'scenario_framework/event.schema.json').read_text())
        for event in events:validate(event,schema)
        return result,calls,events

    def test_service_order_and_monitor_attached_before_commands(self):
        result,calls,events=self.execute()
        self.assertEqual(result['execution_status'],'COMPLETED')
        self.assertEqual([name for name,_ in calls[:3]],['/autonomy/preflight_check','/autonomy/set_local_destination','/autonomy/enable'])
        self.assertEqual(calls[1][1],{'north':8.,'east':0.,'down':-5.})
        enable_seq=next(e['seq'] for e in events if e['kind']=='service_response' and e['source']=='/autonomy/enable')
        self.assertTrue(any(e['kind']=='state' and e['seq']<enable_seq for e in events))
        self.assertEqual([e['seq'] for e in events],list(range(1,len(events)+1)))

    def test_abnormal_terminal_state_remains_execution_completed(self):
        for state in ('HOLD','ABORT'):
            with self.subTest(state=state):
                r,_,_=self.execute(terminal=state)
                self.assertEqual((r['execution_status'],r['last_state']),('COMPLETED',state))
                self.assertEqual(r['verdict'],'FAIL')

    def test_mission_timeout(self):
        r,_,_=self.execute(terminal='NAVIGATING')
        self.assertEqual((r['execution_status'],r['reason'],r['phase']),('TIMED_OUT','TIMEOUT','mission'))

    def test_monitor_loss_is_error_not_expectation_failure(self):
        r,_,_=self.execute(terminal='NAVIGATING',loss=True)
        self.assertEqual((r['execution_status'],r['reason']),('ERROR','MONITOR_DATA_MISSING'))

    def test_failed_preflight_never_enables(self):
        r,calls,_=self.execute(preflight_ok=False)
        self.assertEqual((r['execution_status'],r['reason'],r['phase']),('ERROR','TIMEOUT','startup'))
        self.assertTrue(all(name.endswith('preflight_check') for name,_ in calls))

    def test_discover_path_publisher_before_mission(self):
        r,calls,_=self.execute(no_path_publisher=True)
        self.assertEqual(r['execution_status'],'ERROR');self.assertFalse(calls)

    def test_rejected_commands(self):
        for reject in ('destination','enable'):
            with self.subTest(reject=reject):
                r,calls,_=self.execute(reject=reject)
                self.assertEqual((r['execution_status'],r['reason']),('ERROR','INFRASTRUCTURE_ERROR'))
                if reject=='destination':self.assertFalse(any(name.endswith('enable') for name,_ in calls))

    def test_path_and_position_payloads_preserve_clock_domains(self):
        _,_,events=self.execute()
        path=next(e for e in events if e['kind']=='path')
        self.assertEqual(path['payload']['points'],[[1.,2.,-5.]])
        self.assertAlmostEqual(path['ros_time_s'],1.000000002)
        pos=next(e for e in events if e['kind']=='position')
        self.assertIsNone(pos['ros_time_s'])
        self.assertEqual(pos['payload']['source_clock'],'px4_boot')
        self.assertEqual(pos['payload']['source_stamp_s'],1.)


if __name__=='__main__':unittest.main()
