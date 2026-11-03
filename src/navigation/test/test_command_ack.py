"""Regression tests for PX4 command acknowledgement ownership."""

import ast
from pathlib import Path
from types import MethodType, SimpleNamespace


class VehicleCommand:
    pass


class VehicleCommandAck:
    VEHICLE_CMD_RESULT_ACCEPTED = 0
    VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED = 1
    VEHICLE_CMD_RESULT_FAILED = 4
    VEHICLE_CMD_RESULT_IN_PROGRESS = 5


def acknowledgement(command, result, from_external=False, target=(1, 1)):
    return SimpleNamespace(
        command=command,
        result=result,
        from_external=from_external,
        target_system=target[0],
        target_component=target[1],
    )


def command_node():
    source = Path(__file__).parents[1] / "navigation/mission_node.py"
    mission_class = next(
        node
        for node in ast.parse(source.read_text()).body
        if isinstance(node, ast.ClassDef) and node.name == "MissionNode"
    )
    methods = [
        node
        for node in mission_class.body
        if isinstance(node, ast.FunctionDef)
        and node.name in {"_on_command_ack", "_command"}
    ]
    module = ast.Module(
        body=[
            ast.ImportFrom(
                module="__future__",
                names=[ast.alias(name="annotations")],
                level=0,
            ),
            *methods,
        ],
        type_ignores=[],
    )
    scope = {
        "VehicleCommand": VehicleCommand,
        "VehicleCommandAck": VehicleCommandAck,
        "ACTIVE_STATES": {"NAVIGATING"},
    }
    exec(compile(ast.fix_missing_locations(module), str(source), "exec"), scope)

    node = SimpleNamespace(
        pending_commands=set(),
        last_command_ack=None,
        state="NAVIGATING",
        published=[],
        aborts=[],
    )
    node.command_pub = SimpleNamespace(
        publish=lambda message: node.published.append(message)
    )
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=123000)
    )
    node.get_logger = lambda: SimpleNamespace(debug=lambda *args: None)
    node._safe_abort = lambda reason: node.aborts.append(reason)
    node._command = MethodType(scope["_command"], node)
    node._on_command_ack = MethodType(scope["_on_command_ack"], node)
    return node


def test_command_is_recorded_before_acknowledgement():
    node = command_node()
    node._command(400)

    assert 400 in node.pending_commands
    assert len(node.published) == 1


def test_external_rejection_for_pending_command_is_ignored():
    node = command_node()
    node._command(400)

    node._on_command_ack(
        acknowledgement(
            400,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
            from_external=True,
        )
    )

    assert 400 in node.pending_commands
    assert not node.aborts
    assert node.last_command_ack is None


def test_ground_station_rejection_of_pending_command_is_ignored():
    node = command_node()
    node._command(400)

    node._on_command_ack(
        acknowledgement(
            400,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
            target=(255, 190),
        )
    )

    assert 400 in node.pending_commands
    assert not node.aborts


def test_untracked_internal_rejection_is_ignored():
    node = command_node()

    node._on_command_ack(
        acknowledgement(
            512,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
        )
    )

    assert not node.aborts
    assert node.last_command_ack is None


def test_owned_rejection_aborts_active_mission():
    node = command_node()
    node._command(400)

    node._on_command_ack(
        acknowledgement(
            400,
            VehicleCommandAck.VEHICLE_CMD_RESULT_FAILED,
        )
    )

    assert 400 not in node.pending_commands
    assert node.aborts


def test_owned_acceptance_clears_pending_command():
    node = command_node()
    node._command(400)

    node._on_command_ack(
        acknowledgement(
            400,
            VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED,
        )
    )

    assert 400 not in node.pending_commands
    assert node.last_command_ack == (400, 0)
    assert not node.aborts


def test_in_progress_command_remains_pending():
    node = command_node()
    node._command(400)

    node._on_command_ack(
        acknowledgement(
            400,
            VehicleCommandAck.VEHICLE_CMD_RESULT_IN_PROGRESS,
        )
    )

    assert 400 in node.pending_commands
    assert not node.aborts
