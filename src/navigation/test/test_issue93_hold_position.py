"""Exercise real mission methods without requiring a ROS installation."""
import ast
import math
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest

from navigation.state import MissionState, can_transition


def mission():
    source = Path(__file__).parents[1] / 'navigation/mission_node.py'
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
    names = {'_transition', '_cruise_z', '_flight_z', '_preflight', '_takeoff', '_navigate', '_approach',
             '_landing_check'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    scope = dict(math=math, MissionState=MissionState, can_transition=can_transition,
                 VehicleCommand=SimpleNamespace(VEHICLE_CMD_DO_SET_MODE=176,
                     VEHICLE_CMD_COMPONENT_ARM_DISARM=400, VEHICLE_CMD_NAV_LAND=21))
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), scope)
    parameters = dict(cruise_altitude_m=5, require_lidar=True, arrival_tolerance_m=0.75,
                      landing_confirmations=8, landing_check_timeout_s=5)
    node = SimpleNamespace(state=MissionState.IDLE, position=(0., 0., 0.),
                           takeoff_point=None, landing_check_point=None,
                           yaw=0., offboard_warmup=0, landing_confirmations=0,
                           state_started=10., now=10., targets=[], commands=[], aborts=[])
    node._now = lambda: node.now
    node._health_ok = lambda: True
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(info=lambda message: None)
    node._setpoint = lambda *args: node.targets.append(args)
    node._command = lambda command, **kwargs: node.commands.append((command, kwargs))
    node._next_search_destination = lambda: False
    node._safe_abort = node.aborts.append
    for method in methods:
        setattr(node, method.name, MethodType(scope[method.name], node))
    return node, parameters


def test_takeoff_target_remains_fixed_during_drift():
    node, _ = mission()
    node.state = MissionState.TAKEOFF
    for position in [(0., 0., 0.), (1.5, -.8, -1.), (2., -1., -3.)]:
        node.position = position
        node._takeoff()
    assert [target[:3] for target in node.targets] == [(0., 0., -5.)] * 3
    assert node.state == MissionState.TAKEOFF
    node.position = (2., -1., -4.5)
    node._takeoff()
    assert node.state == MissionState.PLANNING


def test_landing_check_target_remains_fixed_during_drift():
    node, _ = mission()
    node.state = MissionState.APPROACH
    node.position = (8., 0., -5.)
    node._transition(MissionState.LANDING_CHECK)
    # Drift before the first handler invocation must not move the entry anchor.
    for position in [(9.2, -.6, -5.), (10., -1., -4.5), (8.5, -.2, -5.)]:
        node.position = position
        node._landing_check()
    assert [target[:3] for target in node.targets] == [(8., 0., -5.)] * 3
    assert node.state == MissionState.LANDING_CHECK
    assert node.commands == []


def test_preflight_anchor_survives_warmup_and_takeoff():
    node, _ = mission()
    node._transition(MissionState.PREFLIGHT)
    node.position = None
    node._preflight()
    assert node.takeoff_point is None and not node.targets
    node.position = (3., 4., 0.)
    node._health_ok = lambda: False
    node._preflight()
    assert node.takeoff_point is None and not node.targets
    node._health_ok = lambda: True
    node._preflight()
    for _ in range(9):
        node.position = (4., 5., -1.)
        node._preflight()
    assert node.state == MissionState.TAKEOFF
    assert node.commands == [(176, dict(param1=1., param2=6.)), (400, dict(param1=1.))]
    node._takeoff()
    assert [target[:3] for target in node.targets] == [(3., 4., -5.)] * 11


@pytest.mark.parametrize('previous', [MissionState.COMPLETE, MissionState.ABORT])
def test_new_preflight_resets_anchors(previous):
    node, _ = mission()
    node.state = previous
    node.takeoff_point = node.landing_check_point = (99., 99., -5.)
    node._transition(MissionState.PREFLIGHT)
    assert node.takeoff_point is None and node.landing_check_point is None
    node._preflight()
    assert node.takeoff_point == (0., 0., 0.)


def test_new_landing_check_refreshes_anchor_and_confirmations():
    node, _ = mission()
    for position in [(8., 0., -5.), (9., 2., -5.)]:
        node.state = MissionState.APPROACH
        node.position = position
        node.landing_confirmations = 7
        node._transition(MissionState.LANDING_CHECK)
        assert node.landing_check_point == position
        assert node.landing_confirmations == 0
        node._landing_check()
        assert node.targets[-1][:3] == position


def test_landing_timeout_still_searches_then_aborts_when_exhausted():
    node, _ = mission()
    node.state = MissionState.APPROACH
    node._transition(MissionState.LANDING_CHECK)
    searches = []
    node._next_search_destination = lambda: searches.append(True) or True
    node.now = 15.
    node._landing_check()
    assert not searches
    node.now = 15.1
    node._landing_check()
    assert searches == [True] and not node.aborts
    node._next_search_destination = lambda: False
    node._landing_check()
    assert node.aborts == ['no safe landing zone found']


@pytest.mark.parametrize('require_lidar,confirmations', [(False, 0), (True, 8)])
def test_landing_command_preserves_confirmation_and_no_lidar_behavior(require_lidar, confirmations):
    node, parameters = mission()
    node.state = MissionState.APPROACH
    node._transition(MissionState.LANDING_CHECK)
    parameters['require_lidar'] = require_lidar
    node.landing_confirmations = confirmations
    node._landing_check()
    assert node.commands == [(21, {})]
    assert node.state == MissionState.LANDING


def test_cruise_height_is_above_the_pad_not_px4_origin():
    node, _ = mission()
    node._transition(MissionState.PREFLIGHT)
    node.position = (0., 0., 2.)  # estimator has drifted 2 m down while sitting on the pad
    node._preflight()
    assert node.targets[-1][2] == -3.


@pytest.mark.parametrize('down, flown', [(0., -5.), (40., -5.), (-3., -5.), (-12., -12.)])
def test_destination_down_never_flies_below_cruise(down, flown):
    node, _ = mission()
    node.takeoff_point = (0., 0., 0.)
    node.position = (0., 0., -5.)
    node.destination = (8., 0., down)
    node.obstacle_detected = False
    node.path, node.path_index = [(0., 0.), (8., 0.)], 1
    node.state = MissionState.NAVIGATING
    node._navigate()
    assert node.targets[-1][2] == flown
    node.state = MissionState.APPROACH
    node.position = (8., 0., flown)
    node._approach()
    assert node.targets[-1][:3] == (8., 0., flown)
    assert node.state == MissionState.LANDING_CHECK
