"""Exercise real node methods without ROS transport, like the command-ack tests."""

import ast
from dataclasses import dataclass
import json
import math
from pathlib import Path
from types import MethodType, SimpleNamespace as NS

import pytest

from navigation.state import MissionState


@dataclass
class Health:
    received_s: float | None = None
    frame_id: str = ""


def bind(node, filename, class_name, methods):
    source = Path(__file__).parents[1] / "navigation" / filename
    cls = next(n for n in ast.parse(source.read_text()).body
               if isinstance(n, ast.ClassDef) and n.name == class_name)
    selected = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in methods]
    module = ast.Module(body=[ast.ImportFrom(module="__future__",
        names=[ast.alias(name="annotations")], level=0), *selected], type_ignores=[])
    scope = dict(json=json, math=math, TopicHealth=Health, String=NS,
                 VehicleStatus=NS(ARMING_STATE_DISARMED=1),
                 BatteryStatus=NS(BATTERY_WARNING_NONE=0), SensorGps=NS(FIX_TYPE_3D=3),
                 MissionState=MissionState, ACTIVE_STATES={MissionState.HOLD, MissionState.NAVIGATING})
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), scope)
    for name in methods:
        setattr(node, name, MethodType(scope[name], node))
    return node


def diagnostics():
    node = NS(time=100.0, published=[])
    node._now = lambda: node.time
    node.get_parameter = lambda name: NS(value={"diagnostic_timeout_s": 2.0,
                                               "require_lidar": True}[name])
    node.health = {name: Health(100.0, "sensor") for name in (
        "oakd_depth_image", "oakd_depth_points", "lidar_points", "px4_status",
        "px4_position", "battery", "failsafe_flags", "gps")}
    node.vehicle_status = NS(pre_flight_checks_pass=True, arming_state=1)
    node.battery_status = NS(connected=True, warning=0)
    node.failsafe_flags = NS(manual_control_signal_lost=False,
        battery_low_remaining_time=False, battery_unhealthy=False,
        local_position_invalid=False, global_position_invalid=False,
        home_position_invalid=False, battery_warning=0)
    node.gps = NS(fix_type=3, satellites_used=8, eph=2.49)
    node.preflight_result_pub = NS(publish=node.published.append)
    return bind(node, "diagnostics_node.py", "DiagnosticsNode",
                {"_check", "_on_status", "_on_battery", "_on_failsafe_flags", "_on_gps"})


def test_healthy_check_publishes_pass_with_original_time():
    node = diagnostics()
    assert node._check(None, NS()).success
    assert json.loads(node.published[-1].data) == {"success": True, "checked_at_s": 100.0}


@pytest.mark.parametrize("message,field,value,expected", [
    ("vehicle_status", "pre_flight_checks_pass", False, "PX4 preflight"),
    ("vehicle_status", "arming_state", 2, "not disarmed"),
    ("battery_status", "connected", False, "not connected"),
    ("battery_status", "warning", 1, "battery warning"),
    ("failsafe_flags", "manual_control_signal_lost", True, "RC signal lost"),
    ("failsafe_flags", "battery_low_remaining_time", True, "flight time low"),
    ("failsafe_flags", "battery_unhealthy", True, "battery unhealthy"),
    ("failsafe_flags", "local_position_invalid", True, "local position invalid"),
    ("failsafe_flags", "global_position_invalid", True, "global position invalid"),
    ("failsafe_flags", "home_position_invalid", True, "home position invalid"),
    ("failsafe_flags", "battery_warning", 1, "failsafe: battery warning"),
    ("gps", "fix_type", 2, "3D fix"),
    ("gps", "satellites_used", 7, "8 satellites"),
    *[("gps", "eph", value, "GPS eph") for value in (2.5, 3.0, -1.0, math.nan, math.inf)],
])
def test_each_failure_is_reported(message, field, value, expected):
    node = diagnostics()
    setattr(getattr(node, message), field, value)
    response = node._check(None, NS())
    assert not response.success
    assert expected in response.message
    assert json.loads(node.published[-1].data)["success"] is False


def test_multiple_failures_are_all_reported():
    node = diagnostics()
    node.battery_status.connected = False
    node.battery_status.warning = 1
    node.gps.satellites_used = 2
    response = node._check(None, NS())
    assert not response.success
    for text in ("not connected", "battery warning", "8 satellites"):
        assert text in response.message


@pytest.mark.parametrize("topic", ["px4_status", "battery", "failsafe_flags", "gps"])
@pytest.mark.parametrize("stamp", [None, 97.9])
def test_missing_or_stale_px4_topics_fail(topic, stamp):
    node = diagnostics()
    node.health[topic].received_s = stamp
    response = node._check(None, NS())
    assert not response.success
    assert topic in response.message


@pytest.mark.parametrize("method,attribute,topic", [
    ("_on_status", "vehicle_status", "px4_status"),
    ("_on_battery", "battery_status", "battery"),
    ("_on_failsafe_flags", "failsafe_flags", "failsafe_flags"),
    ("_on_gps", "gps", "gps"),
])
def test_callbacks_keep_message_and_arrival_time(method, attribute, topic):
    node = diagnostics()
    msg = NS()
    node.time = 101.0
    getattr(node, method)(msg)
    assert getattr(node, attribute) is msg
    assert node.health[topic].received_s == 101.0


def mission():
    node = NS(time=100.0, preflight_passed_at=None, state=MissionState.IDLE,
              destination=(1, 0, -5), original_destination=(1, 0, -5), path=[])
    node._now = lambda: node.time
    node._transition = lambda state: setattr(node, "state", state)
    node.get_logger = lambda: NS(warning=lambda message: None)
    return bind(node, "mission_node.py", "MissionNode", {"_on_preflight_result", "_enable"})


@pytest.mark.parametrize("stamp,allowed", [(None, False), (69.99, False),
    (70.0, True), (100.0, True), (101.0, False)])
def test_enable_requires_recent_pass(stamp, allowed):
    node = mission()
    node.preflight_passed_at = stamp
    response = node._enable(NS(data=True), NS())
    assert response.success is allowed
    assert node.state == (MissionState.PREFLIGHT if allowed else MissionState.IDLE)
    if allowed:
        assert node.preflight_passed_at is None


@pytest.mark.parametrize("payload", ["broken", "{}", "null", "[]",
    '{"success":false,"checked_at_s":100}',
    '{"success":true,"checked_at_s":69}',
    '{"success":true,"checked_at_s":101}',
    '{"success":true,"checked_at_s":"NaN"}'])
def test_invalid_failed_or_expired_result_clears_pass(payload):
    node = mission()
    node.preflight_passed_at = 99.0
    node._on_preflight_result(NS(data=payload))
    assert node.preflight_passed_at is None
    assert not node._enable(NS(data=True), NS()).success


def test_published_result_allows_start_then_expires():
    diagnostic = diagnostics()
    diagnostic._check(None, NS())
    node = mission()
    node._on_preflight_result(diagnostic.published[-1])
    assert node.preflight_passed_at == 100.0
    node.time = 130.01
    assert not node._enable(NS(data=True), NS()).success


def test_hold_resume_does_not_require_disarmed_preflight():
    node = mission()
    node.state = MissionState.HOLD
    node.operator_hold = True
    assert node._enable(NS(data=True), NS()).success
    assert not node.operator_hold
