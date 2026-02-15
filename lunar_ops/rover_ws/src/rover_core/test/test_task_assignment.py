import inspect
import trace

import pytest

from _helpers import FakeBus, load_module


@pytest.fixture()
def earth_node(monkeypatch):
    bus = FakeBus()
    earth_module = load_module(monkeypatch, 'rover_core.earth_node', bus)
    return earth_module.EarthNode()


def test_select_best_rover_prefers_highest_battery_when_idle(earth_node):
    available = {
        'rover_1': {'state': 'IDLE', 'battery': 0.72, 'solar_exposure': 0.8},
        'rover_2': {'state': 'IDLE', 'battery': 0.91, 'solar_exposure': 0.7},
        'rover_3': {
            'state': 'EXECUTING',
            'battery': 0.99,
            'solar_exposure': 0.9,
        },
    }

    assert earth_node.select_best_rover(available) == 'rover_2'


def test_select_best_rover_considers_solar_exposure_in_scoring(earth_node):
    available = {
        'rover_low_solar': {
            'state': 'IDLE',
            'battery': 0.95,
            'solar_exposure': 0.2,
        },
        'rover_high_solar': {
            'state': 'IDLE',
            'battery': 0.55,
            'solar_exposure': 0.9,
        },
    }

    # Scores:
    # rover_low_solar  = 0.95*0.5 + 0.3 = 0.775
    # rover_high_solar = 0.55*0.5 + 1.0 = 1.275
    assert earth_node.select_best_rover(available) == 'rover_high_solar'


def test_auto_assign_rejects_when_all_rovers_unavailable(
    earth_node, monkeypatch
):
    earth_node.fleet_registry = {
        'rover_1': {'state': 'EXECUTING', 'battery': 0.8, 'solar_exposure': 0.7},
        'rover_2': {'state': 'SAFE_MODE', 'battery': 0.9, 'solar_exposure': 0.9},
    }

    dispatched = []
    monkeypatch.setattr(
        earth_node,
        'send_command',
        lambda rover_id, cmd_type, task_id=None: dispatched.append(
            (rover_id, cmd_type, task_id)
        ),
    )

    assert earth_node.auto_assign_task('task-no-idle') is False
    assert dispatched == []


def test_auto_assign_edge_case_single_rover(earth_node, monkeypatch):
    earth_node.fleet_registry = {
        'rover_1': {'state': 'IDLE', 'battery': 0.05, 'solar_exposure': 0.1}
    }

    dispatched = []
    monkeypatch.setattr(
        earth_node,
        'send_command',
        lambda rover_id, cmd_type, task_id=None: dispatched.append(
            (rover_id, cmd_type, task_id)
        ),
    )

    assert earth_node.auto_assign_task('task-single') is True
    assert dispatched == [('rover_1', 'START_TASK', 'task-single')]


def test_auto_assign_edge_case_all_low_battery(earth_node, monkeypatch):
    earth_node.fleet_registry = {
        'rover_1': {'state': 'IDLE', 'battery': 0.08, 'solar_exposure': 0.3},
        'rover_2': {'state': 'IDLE', 'battery': 0.04, 'solar_exposure': 0.9},
        'rover_3': {'state': 'IDLE', 'battery': 0.09, 'solar_exposure': 0.1},
    }

    dispatched = []
    monkeypatch.setattr(
        earth_node,
        'send_command',
        lambda rover_id, cmd_type, task_id=None: dispatched.append(
            (rover_id, cmd_type, task_id)
        ),
    )

    assert earth_node.auto_assign_task('task-low-battery') is True
    assert dispatched == [('rover_2', 'START_TASK', 'task-low-battery')]


def test_manual_assignment_overrides_auto_selection(earth_node, monkeypatch):
    earth_node.fleet_registry = {
        'rover_1': {'state': 'IDLE', 'battery': 0.95, 'solar_exposure': 0.8},
        'rover_2': {'state': 'SAFE_MODE', 'battery': 0.4, 'solar_exposure': 0.2},
    }

    dispatched = []

    def record_send(rover_id, cmd_type, task_id=None):
        dispatched.append((rover_id, cmd_type, task_id))

    monkeypatch.setattr(earth_node, 'send_command', record_send)

    assert earth_node.auto_assign_task('task-auto') is True
    earth_node.assign_task('rover_2', 'task-manual')

    assert dispatched[0] == ('rover_1', 'START_TASK', 'task-auto')
    assert dispatched[1] == ('rover_2', 'START_TASK', 'task-manual')


def _tracked_lines(func):
    source_lines, start_line = inspect.getsourcelines(func)
    tracked = set()
    for offset, text in enumerate(source_lines):
        stripped = text.strip()
        if not stripped:
            continue
        if stripped.startswith('#'):
            continue
        if stripped.startswith('def '):
            continue
        if stripped.startswith('"""') or stripped.startswith("'''"):
            continue
        tracked.add(start_line + offset)
    return tracked


def _exercise_assignment_paths(node):
    node.select_best_rover(
        {
            'rover_1': {'state': 'IDLE', 'battery': 0.7, 'solar_exposure': 0.6},
            'rover_2': {
                'state': 'EXECUTING',
                'battery': 1.0,
                'solar_exposure': 0.9,
            },
        }
    )
    node.select_best_rover(
        {
            'rover_1': {'state': 'SAFE_MODE', 'battery': 0.3, 'solar_exposure': 0.2}
        }
    )

    node.send_command = lambda _rid, _cmd, _task=None: None
    node.fleet_registry = {
        'rover_1': {'state': 'IDLE', 'battery': 0.8, 'solar_exposure': 0.8}
    }
    node.auto_assign_task('task-success')

    node.fleet_registry = {
        'rover_1': {
            'state': 'EXECUTING',
            'battery': 0.8,
            'solar_exposure': 0.8,
        }
    }
    node.auto_assign_task('task-rejected')
    node.assign_task('rover_2', 'task-manual')


def test_assignment_logic_line_coverage_above_80_percent(earth_node):
    tracer = trace.Trace(count=True, trace=False)
    tracer.runfunc(_exercise_assignment_paths, earth_node)
    counts = tracer.results().counts

    source_file = earth_node.__class__.select_best_rover.__code__.co_filename
    tracked_lines = set()
    tracked_lines.update(_tracked_lines(earth_node.__class__.select_best_rover))
    tracked_lines.update(_tracked_lines(earth_node.__class__.auto_assign_task))
    tracked_lines.update(_tracked_lines(earth_node.__class__.assign_task))

    executed = sum(1 for line in tracked_lines if (source_file, line) in counts)
    coverage_ratio = executed / len(tracked_lines)

    assert coverage_ratio >= 0.80
