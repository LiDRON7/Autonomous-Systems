"""Execute actual MissionNode methods with ROS transport replaced by small fakes.

This checks callback/planning/timer wiring without importing unavailable ROS packages.
It does not replace ROS integration or verify the incoming cloud convention.
"""
import ast
import os
from pathlib import Path
from types import MethodType, SimpleNamespace

import numpy as np

from navigation.grid import RollingGrid
from navigation.mapping import depth_points_to_world
from navigation.planner import astar, simplify
from navigation.state import MissionState


def mission_methods(**overrides):
    source = Path(os.environ.get('ISSUE73_MISSION_SOURCE', Path(__file__).parents[1] / 'navigation/mission_node.py'))
    tree = ast.parse(source.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'MissionNode')
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in {
        '_on_depth_points', '_refresh_grid', '_plan', '_planning_goal', '_tick',
    }]
    module = ast.Module(body=[ast.ImportFrom(module='__future__', names=[ast.alias(name='annotations')], level=0), *methods], type_ignores=[])
    scope = dict(np=np, math=__import__("math"), point_cloud2=SimpleNamespace(read_points_numpy=lambda msg, **kw: msg.points),
                 depth_points_to_world=depth_points_to_world, astar=astar, simplify=simplify,
                 MissionState=MissionState, ACTIVE_STATES=set(), String=lambda **kw: kw)
    exec(compile(ast.fix_missing_locations(module), str(source), 'exec'), scope)
    parameters = {'map.min_height_m': -1.5, 'map.max_height_m': 1.5,
                  'map.sensor_translation_flu_m': [0.0, 0.0, 0.0]}
    node = SimpleNamespace(position=(0, 0, -5), yaw=0.0, now=10.0, stamps={},
                           grid=RollingGrid(inflation_m=0.0), grid_origin=(0, 0),
                           state=MissionState.IDLE, mission_started=None,
                           destination=(6, 0, -5), published=[], path_published=[],
                           state_pub=SimpleNamespace(publish=lambda msg: None))
    node._now = lambda: node.now
    node.get_parameter = lambda name: SimpleNamespace(value=parameters[name])
    node.get_logger = lambda: SimpleNamespace(debug=lambda *args, **kw: None)
    node._publish_grid = lambda: node.published.append((node.grid_origin, dict(node.grid.occupied)))
    node._publish_path = lambda: node.path_published.append(node.path)
    node._transition = lambda state: setattr(node, 'state', state)
    # _tick constructs its handler table even while testing one branch.
    for name in ['_preflight', '_takeoff', '_navigate', '_approach', '_landing_check',
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
