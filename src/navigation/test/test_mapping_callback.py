"""Execute actual MissionNode methods with ROS transport replaced by small fakes.

This checks callback/planning/timer wiring without importing unavailable ROS packages.
It does not replace ROS integration or verify the incoming cloud convention.
"""
import ast
import os
from itertools import chain
from pathlib import Path
from types import MethodType, SimpleNamespace

import numpy as np

from navigation.grid import RollingGrid
from navigation.mapping import depth_points_to_world
from navigation.planner import astar, escape_leg_clear, find_escape_target, simplify
from navigation.state import MissionState, data_is_fresh


def mission_methods(**overrides):
    source = Path(os.environ.get('ISSUE73_MISSION_SOURCE', Path(__file__).parents[1] / 'navigation/mission_node.py'))
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in {
        '_on_depth_points', '_refresh_grid', '_plan', '_planning_goal', '_tick',
        '_navigate', '_hold', '_on_obstacle', '_detour_ready', '_obstacle_reading_fresh',
    }]
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), *methods], type_ignores=[])
    scope = dict(np=np, math=__import__("math"), point_cloud2=SimpleNamespace(read_points_numpy=lambda msg, **kw: msg.points),
                 depth_points_to_world=depth_points_to_world, astar=astar, simplify=simplify,
                 chain=chain, escape_leg_clear=escape_leg_clear,
                 find_escape_target=find_escape_target, data_is_fresh=data_is_fresh,
                 MissionState=MissionState, ACTIVE_STATES=set(), String=lambda **kw: kw)
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), scope)
    parameters = {'map.min_height_m': -1.5, 'map.max_height_m': 1.5,
                  'map.sensor_translation_flu_m': [0.0, 0.0, 0.0],
                  'arrival_tolerance_m': 0.75, 'sensor_timeout_s': 1.0}
    node = SimpleNamespace(position=(0, 0, -5), yaw=0.0, now=10.0, stamps={},
                           grid=RollingGrid(inflation_m=0.0), grid_origin=(0, 0),
                           state=MissionState.IDLE, mission_started=None,
                           destination=(6, 0, -5), published=[], path_published=[],
                           obstacle_detected=False, obstacle_stamp=None,
                           align_detour=False, setpoints=[],
                           state_pub=SimpleNamespace(publish=lambda msg: None))
    node._now = lambda: node.now
    node._setpoint = lambda *target: node.setpoints.append(target)
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(debug=lambda *args, **kw: None)
    node._publish_grid = lambda: node.published.append((node.grid_origin, dict(node.grid.occupied)))
    node._publish_path = lambda: node.path_published.append(node.path)
    node._transition = lambda state: setattr(node, 'state', state)
    # _tick constructs its handler table even while testing one branch.
    for name in ['_preflight', '_takeoff', '_approach', '_landing_check',
                 '_recover_from_hold', '_wait_for_landing']:
        setattr(node, name, lambda: None)
    for name in [m.name for m in methods]:
        setattr(node, name, MethodType(scope[name], node))
    node.__dict__.update(overrides)
    return node


def test_actual_callback_inserts_expected_obstacle_and_stamps_depth():
    node = mission_methods()
    node._on_depth_points(SimpleNamespace(points=[[3, 1, 0]]))
    cell = node.grid.to_cell((3, -1), node.grid_origin)
    assert cell in node.grid.occupied
    assert node.stamps['depth'] == 10.0
    assert len(node.published) == 1


def test_callback_without_position_does_not_claim_fresh_depth():
    node = mission_methods(position=None)
    node._on_depth_points(SimpleNamespace(points=[[3, 0, 0]]))
    assert not node.stamps
    assert not node.grid.occupied


def test_plan_reprojects_cells_after_movement_between_clouds():
    node = mission_methods()
    node.grid_origin = node.grid.origin_around(node.position[:2])
    node.grid.observe([(3, 0)], node.grid_origin, node.now)
    old_cell = node.grid.to_cell((3, 0), node.grid_origin)
    node.position = (1, 0, -5)
    node._plan()
    new_cell = node.grid.to_cell((3, 0), node.grid_origin)
    assert new_cell != old_cell
    assert new_cell in node.grid.occupied
    assert old_cell not in node.grid.occupied
    assert node.state == MissionState.NAVIGATING
    assert node.path_published


def test_timer_expires_and_publishes_without_cloud_callbacks():
    node = mission_methods()
    node.grid_origin = node.grid.origin_around(node.position[:2])
    node.grid.observe([(3, 0)], node.grid_origin, node.now)
    node.stamps['depth'] = node.now
    node.now = 12.0
    node._tick()
    assert node.grid.occupied  # Exact expiry boundary is inclusive.
    node.now = 12.01
    node._tick()
    assert not node.grid.occupied
    assert not node.grid.observations
    assert node.published[-1][1] == {}
    assert node.stamps['depth'] == 10.0  # Timer must not claim a new cloud.

def close_obstacle_mission():
    node = mission_methods(
        grid=RollingGrid(size_m=10.0, inflation_m=1.25),
        state=MissionState.REPLANNING,
        destination=(3.0, 0.0, -5.0),
        obstacle_detected=True,
        obstacle_stamp=1.0,
        now=1.0,
    )
    node.grid_origin = node.grid.origin_around(node.position[:2])
    return node


def test_detour_holds_position_until_clear_reading_after_turn():
    node = close_obstacle_mission()
    node.grid.observe([(1.5, 0.0)], node.grid_origin, node.now)
    node._plan()
    assert node.state == MissionState.NAVIGATING
    assert not node.escaping_buffer

    node._navigate()
    assert node.setpoints[-1][:3] == node.position
    node.yaw = node.setpoints[-1][3]
    node._on_obstacle(SimpleNamespace(data=False))  # Received before alignment.
    node.now = 1.1
    node._navigate()
    node._navigate()
    assert node.setpoints[-1][:3] == node.position

    node.now = 1.2
    node._on_obstacle(SimpleNamespace(data=True))
    node._navigate()
    assert node.setpoints[-1][:3] == node.position

    node.now = 1.3
    node._on_obstacle(SimpleNamespace(data=False))
    node._navigate()
    assert node.setpoints[-1][:2] == node.path[1]


def test_inflated_start_commands_the_exit_then_plans_again():
    node = close_obstacle_mission()
    node.grid.observe([(0.75, 0.0)], node.grid_origin, node.now)
    node._plan()
    assert node.escaping_buffer
    assert node.path_index == 0
    exit_target = node.path[0]
    assert exit_target[0] < 0.0
    assert escape_leg_clear(node.position[:2], exit_target, node.grid.observations, node.grid)

    node._navigate()
    assert node.setpoints[-1][:3] == node.position
    node.yaw = node.setpoints[-1][3]
    node._navigate()
    node.now = 1.1
    node._on_obstacle(SimpleNamespace(data=False))
    node._navigate()
    assert node.setpoints[-1][:3] == (*exit_target, node.position[2])

    node.position = (*exit_target, node.position[2])
    node._navigate()
    node._navigate()
    assert node.state == MissionState.PLANNING


def test_observed_start_cell_keeps_drone_in_hold():
    node = close_obstacle_mission()
    node.grid.observe([(0.1, 0.1)], node.grid_origin, node.now)
    node._plan()
    assert node.state == MissionState.HOLD
    assert node.setpoints[-1][:3] == node.position


def test_escape_segment_cannot_pass_through_an_obstacle():
    grid = RollingGrid(inflation_m=1.25)
    assert not escape_leg_clear((0.0, 0.0), (2.5, 0.0), [(0.75, 0.0)], grid)


def test_new_obstacle_blocks_escape_after_planning():
    node = close_obstacle_mission()
    node.grid.observe([(0.75, 0.0)], node.grid_origin, node.now)
    node._plan()
    node.grid.observe([node.path[0]], node.grid_origin, node.now)
    node._navigate()
    assert node.state == MissionState.HOLD
    assert node.setpoints[-1][:3] == node.position
