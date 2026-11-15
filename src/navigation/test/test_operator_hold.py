import ast
from pathlib import Path
from types import MethodType, SimpleNamespace

from navigation.state import MissionState

def make_mission():
    source = Path(__file__).parents[1] / "navigation/mission_node.py"
    tree = ast.parse(source.read_text())

    mission_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "MissionNode"
    )

    methods = [
        item for item in mission_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name in {"_enable", "_recover_from_hold"}
    ]

    module = ast.Module(body=methods, type_ignores=[])
    scope = {
        "MissionState": MissionState,
        "ACTIVE_STATES": {MissionState.NAVIGATING, MissionState.HOLD},
    }
    exec(compile(module, str(source), "exec"), scope)

    node = SimpleNamespace(
        state=MissionState.NAVIGATING,
        operator_hold=False,
        obstacle_detected=False,
    )
    node._health_ok = lambda: True
    node._hold = lambda: None
    node._transition = lambda state: setattr(node, "state", state)

    for method in methods:
        setattr(node, method.name, MethodType(scope[method.name], node))

    return node


def test_operator_hold_does_not_automatically_resume():
    node = make_mission()

    request = SimpleNamespace(data=False)
    response = SimpleNamespace()

    node._enable(request, response)

    assert response.success
    assert node.state == MissionState.HOLD
    assert node.operator_hold is True

    for _ in range(300):
        node._recover_from_hold()
        assert node.state == MissionState.HOLD

def test_operator_can_resume_from_hold():
    node = make_mission()

    node._enable(SimpleNamespace(data=False), SimpleNamespace())
    response = SimpleNamespace()
    node._enable(SimpleNamespace(data=True), response)

    assert response.success
    assert node.operator_hold is False

    node._recover_from_hold()

    assert node.state == MissionState.REPLANNING

def test_resume_waits_for_healthy_data():
    node = make_mission()

    node._enable(SimpleNamespace(data=False), SimpleNamespace())
    node._health_ok = lambda: False

    node._enable(SimpleNamespace(data=True), SimpleNamespace())
    node._recover_from_hold()

    assert node.state == MissionState.HOLD

    node._health_ok = lambda: True
    node._recover_from_hold()

    assert node.state == MissionState.REPLANNING

def test_resume_waits_for_obstacle_to_clear():
    node = make_mission()

    node._enable(SimpleNamespace(data=False), SimpleNamespace())
    node.obstacle_detected = True

    node._enable(SimpleNamespace(data=True), SimpleNamespace())
    node._recover_from_hold()

    assert node.state == MissionState.HOLD

    node.obstacle_detected = False
    node._recover_from_hold()

    assert node.state == MissionState.REPLANNING

def test_automatic_hold_can_still_recover():
    node = make_mission()
    node.state = MissionState.HOLD

    node._recover_from_hold()

    assert node.state == MissionState.REPLANNING