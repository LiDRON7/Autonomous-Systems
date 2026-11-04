import importlib.util
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest

from navigation.state import MissionState


class VehicleStatus(SimpleNamespace):
    # Match the px4_msgs v1.15.0 definitions pinned by integration/Dockerfile.
    NAVIGATION_STATE_POSCTL = 2
    NAVIGATION_STATE_OFFBOARD = 14
    ARMING_STATE_DISARMED = 1
    ARMING_STATE_ARMED = 2


class VehicleCommand(SimpleNamespace):
    VEHICLE_CMD_NAV_LAND = 21
    VEHICLE_CMD_DO_SET_MODE = 176
    VEHICLE_CMD_COMPONENT_ARM_DISARM = 400


class VehicleCommandAck(SimpleNamespace):
    VEHICLE_CMD_RESULT_ACCEPTED = 0
    VEHICLE_CMD_RESULT_IN_PROGRESS = 5


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class FakeNode:
    def __init__(self, name):
        self.now = 10.0
        self.parameters = {}
        self.logs = []

    def declare_parameter(self, name, value):
        self.parameters[name] = value

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(self.now * 1e9)))

    def get_logger(self):
        return SimpleNamespace(**{
            level: lambda message, **kwargs: self.logs.append(message)
            for level in ("info", "warning", "error", "debug")
        })

    def create_publisher(self, *args):
        return Publisher()

    def create_subscription(self, *args):
        pass

    def create_service(self, *args):
        pass

    def create_timer(self, *args):
        pass


@pytest.fixture
def node(monkeypatch):
    modules = {
        "rclpy": {},
        "rclpy.node": {"Node": FakeNode},
        "rclpy.qos": {
            "QoSProfile": SimpleNamespace,
            "ReliabilityPolicy": SimpleNamespace(BEST_EFFORT=1),
            "DurabilityPolicy": SimpleNamespace(VOLATILE=1),
            "HistoryPolicy": SimpleNamespace(KEEP_LAST=1),
        },
        "geometry_msgs.msg": {"PoseStamped": SimpleNamespace},
        "nav_msgs.msg": {"OccupancyGrid": SimpleNamespace, "Path": SimpleNamespace},
        "px4_msgs.msg": {
            "OffboardControlMode": SimpleNamespace,
            "TrajectorySetpoint": SimpleNamespace,
            "VehicleAttitude": SimpleNamespace,
            "VehicleCommand": VehicleCommand,
            "VehicleCommandAck": VehicleCommandAck,
            "VehicleLandDetected": SimpleNamespace,
            "VehicleLocalPosition": SimpleNamespace,
            "VehicleStatus": VehicleStatus,
        },
        "sensor_msgs.msg": {"PointCloud2": SimpleNamespace},
        "sensor_msgs_py": {"point_cloud2": SimpleNamespace()},
        "std_msgs.msg": {"Bool": SimpleNamespace, "String": SimpleNamespace},
        "std_srvs.srv": {"SetBool": SimpleNamespace, "Trigger": SimpleNamespace},
        "interfaces.srv": {
            "SetGpsDestination": SimpleNamespace, "SetLocalDestination": SimpleNamespace,
        },
    }
    for name, attributes in modules.items():
        module = ModuleType(name)
        module.__dict__.update(attributes)
        monkeypatch.setitem(sys.modules, name, module)
    source = Path(__file__).parents[1] / "navigation" / "mission_node.py"
    spec = importlib.util.spec_from_file_location("navigation._takeover_test", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    controller = module.MissionNode()
    controller._refresh_grid = lambda: None  # Map serialization is covered separately.
    controller._publish_path = lambda: None
    controller.position = (0.0, 0.0, -5.0)
    controller.destination = (8.0, 0.0, -5.0)
    controller.original_destination = controller.destination
    controller.stamps = {key: controller.now for key in controller.stamps}
    controller.state = MissionState.NAVIGATING
    controller.mission_started = controller.now
    return controller


def status(node, mode=VehicleStatus.NAVIGATION_STATE_POSCTL,
           arming=VehicleStatus.ARMING_STATE_ARMED):
    node._on_status(VehicleStatus(nav_state=mode, arming_state=arming))


def takeover(node):
    status(node, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    status(node)


def counts(node):
    return tuple(len(pub.messages) for pub in (
        node.offboard_pub, node.setpoint_pub, node.command_pub,
    ))


def enable(node, value=True):
    return node._enable(SimpleNamespace(data=value), SimpleNamespace())


EXECUTING = [state for state in MissionState if state not in {
    MissionState.IDLE, MissionState.COMPLETE, MissionState.ABORT, MissionState.PILOT_OVERRIDE,
}]


@pytest.mark.parametrize("stage", EXECUTING)
def test_takeover_stops_every_executing_stage(node, stage):
    node.state = stage
    node.path = [(1.0, 0.0)]
    takeover(node)
    assert node.state == MissionState.PILOT_OVERRIDE
    assert node.mission_started is None
    assert not node.path
    assert counts(node) == (0, 0, 0)


def test_initial_position_and_idle_status_do_not_claim_takeover(node):
    node.state = MissionState.IDLE
    status(node, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    assert not node.offboard_seen
    node.state = MissionState.PREFLIGHT
    status(node)
    node._tick()
    assert node.state == MissionState.PREFLIGHT
    assert counts(node) == (1, 1, 0)


def test_no_commands_after_takeover_even_past_timeout_and_recovery(node):
    node.path = [(1.0, 0.0)]
    node._tick()
    before = counts(node)
    assert before == (1, 1, 0)
    takeover(node)
    for elapsed in (0.1, 9.0, 301.0, 600.0):
        node.now = 10.0 + elapsed
        node.stamps = {key: node.now for key in node.stamps}
        status(node, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        status(node)
        node._on_obstacle(SimpleNamespace(data=False))
        node._on_landing_assessment(SimpleNamespace(data='{"suitable": true}'))
        node._on_command_ack(VehicleCommandAck(command=176, result=2, from_external=False,
                                               target_system=1, target_component=1,))
        node._tick()
    assert node.state == MissionState.PILOT_OVERRIDE
    assert counts(node) == before
    assert node.state_pub.messages[-1].data == "PILOT_OVERRIDE"


def test_direct_publishers_and_abort_cannot_bypass_override(node):
    takeover(node)
    node._publish_offboard_mode()
    node._setpoint(0.0, 0.0, -5.0, 0.0)
    node._command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    node._safe_abort("late timeout")
    response = node._abort(None, SimpleNamespace())
    assert not response.success
    assert node.state == MissionState.PILOT_OVERRIDE
    assert counts(node) == (0, 0, 0)


@pytest.mark.parametrize("case", [
    "missing_status", "missing_land", "stale_status", "stale_land",
    "future_status", "future_land", "armed", "airborne", "no_destination",
])
def test_restart_rejects_unconfirmed_ground_state(node, case):
    takeover(node)
    status(node, arming=VehicleStatus.ARMING_STATE_DISARMED)
    node._on_land(SimpleNamespace(landed=True))
    if case == "missing_status":
        node.vehicle_status = None
        node.stamps["status"] = None
    elif case == "missing_land":
        node.land_stamp = None  # Default landed=True is not evidence.
    elif case in {"stale_status", "future_status"}:
        node.stamps["status"] = node.now + (1.0 if case == "future_status" else -2.0)
    elif case in {"stale_land", "future_land"}:
        node.land_stamp = node.now + (1.0 if case == "future_land" else -2.0)
    elif case == "armed":
        status(node)
    elif case == "airborne":
        node._on_land(SimpleNamespace(landed=False))
    else:
        node.destination = None
    assert not enable(node).success
    assert node.state == MissionState.PILOT_OVERRIDE
    assert counts(node) == (0, 0, 0)


def test_explicit_enable_restarts_with_fresh_landed_disarmed_status(node):
    takeover(node)
    status(node, arming=VehicleStatus.ARMING_STATE_DISARMED)
    node._on_land(SimpleNamespace(landed=True))
    node.destination = (9.0, 1.0, -5.0)  # Previous landing-search target.
    node.landing_confirmations = 8
    node.search_index = 3
    assert not enable(node, False).success
    assert node.state == MissionState.PILOT_OVERRIDE
    assert enable(node).success
    assert node.state == MissionState.PREFLIGHT
    assert node.mission_started == node.now
    assert not node.offboard_seen
    assert node.offboard_warmup == node.landing_confirmations == node.search_index == 0
    assert node.destination == node.original_destination
    assert counts(node) == (0, 0, 0)
    status(node)  # Initial Position mode in the new mission is not takeover.
    node._tick()
    assert counts(node) == (1, 1, 0)
    takeover(node)
    assert node.state == MissionState.PILOT_OVERRIDE


def test_destination_can_change_without_unlocking_override(node):
    takeover(node)
    response = node._set_local(
        SimpleNamespace(north=4.0, east=2.0, down=-3.0), SimpleNamespace(),
    )
    assert response.accepted
    assert node.destination == node.original_destination == (4.0, 2.0, -3.0)
    assert node.state == MissionState.PILOT_OVERRIDE
    assert counts(node) == (0, 0, 0)


def test_normal_preflight_takeoff_and_navigation_still_work(node):
    node.state = MissionState.IDLE
    assert enable(node).success
    for _ in range(10):
        status(node)
        node._tick()
    assert node.state == MissionState.TAKEOFF
    assert [msg.command for msg in node.command_pub.messages] == [176, 400]
    status(node, VehicleStatus.NAVIGATION_STATE_OFFBOARD)
    node._tick()
    assert node.state == MissionState.PLANNING
    node._tick()
    assert node.state == MissionState.NAVIGATING


def test_normal_hold_recovers_and_landing_completes(node):
    node.state = MissionState.HOLD
    node._tick()
    assert node.state == MissionState.REPLANNING
    node.state = MissionState.LANDING_CHECK
    node.landing_confirmations = 8
    node._tick()
    assert node.state == MissionState.LANDING
    assert node.command_pub.messages[-1].command == VehicleCommand.VEHICLE_CMD_NAV_LAND
    node.vehicle_status = VehicleStatus(arming_state=VehicleStatus.ARMING_STATE_DISARMED)
    node._tick()
    assert node.state == MissionState.COMPLETE

@pytest.mark.parametrize("reason", ["timeout", "service", "rejected_ack"])
def test_normal_abort_still_sends_land(node, reason):
    if reason == "timeout":
        node.now += 301.0
        node._tick()
    elif reason == "service":
        assert node._abort(None, SimpleNamespace()).success
    else:
        node._command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE)
        node._on_command_ack(VehicleCommandAck(command=VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                                               result=2, from_external=False, target_system=1, target_component=1,))
    assert node.state == MissionState.ABORT
    assert node.command_pub.messages[-1].command == VehicleCommand.VEHICLE_CMD_NAV_LAND
