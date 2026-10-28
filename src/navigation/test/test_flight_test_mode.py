"""Run the real MissionNode no-LiDAR and landing methods with ROS replaced by fakes."""
import ast
from pathlib import Path
from types import MethodType, SimpleNamespace

from navigation.state import MissionState, data_is_fresh

VehicleStatus = SimpleNamespace(ARMING_STATE_DISARMED=1, ARMING_STATE_ARMED=2)
VehicleCommand = SimpleNamespace(VEHICLE_CMD_NAV_LAND=21)


def mission(require_lidar, **state):
    source = Path(__file__).parents[1] / 'navigation/mission_node.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
               and n.name in {'_health_ok', '_landing_check', '_wait_for_landing'}]
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0),
                              *methods], type_ignores=[])
    scope = dict(data_is_fresh=data_is_fresh, MissionState=MissionState,
                 VehicleStatus=VehicleStatus, VehicleCommand=VehicleCommand)
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), scope)
    parameters = {'require_lidar': require_lidar, 'sensor_timeout_s': 1.0,
                  'landing_confirmations': 8, 'landing_check_timeout_s': 5.0}
    node = SimpleNamespace(now=10.0, state=MissionState.LANDING_CHECK, state_started=10.0,
                           position=(0, 0, -5), yaw=0.0, landing_confirmations=0,
                           vehicle_status=None, commands=[],
                           stamps={'position': 10.0, 'depth': 10.0, 'lidar': None, 'status': 10.0})
    node.__dict__.update(state)
    node._now = lambda: node.now
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node._setpoint = lambda *args: None
    node._command = node.commands.append
    node._transition = lambda target: setattr(node, 'state', target)
    for method in methods:
        setattr(node, method.name, MethodType(scope[method.name], node))
    return node


def test_missing_lidar_only_blocks_the_mission_when_required():
    assert not mission(require_lidar=True)._health_ok()
    assert mission(require_lidar=False)._health_ok()


def test_other_stale_inputs_still_block_without_lidar():
    node = mission(require_lidar=False)
    node.stamps['depth'] = None
    assert not node._health_ok()


def test_no_lidar_lands_at_the_destination_without_a_verdict():
    node = mission(require_lidar=False)
    node._landing_check()
    assert node.commands == [21] and node.state == MissionState.LANDING


def test_lidar_mode_still_waits_for_safe_verdicts():
    node = mission(require_lidar=True)
    node._landing_check()
    assert node.commands == [] and node.state == MissionState.LANDING_CHECK


def test_landing_completes_only_after_px4_disarms():
    node = mission(require_lidar=False, state=MissionState.LANDING)
    node._wait_for_landing()
    assert node.state == MissionState.LANDING
    node.vehicle_status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_ARMED)
    node._wait_for_landing()
    assert node.state == MissionState.LANDING
    node.vehicle_status = SimpleNamespace(arming_state=VehicleStatus.ARMING_STATE_DISARMED)
    node._wait_for_landing()
    assert node.state == MissionState.COMPLETE
