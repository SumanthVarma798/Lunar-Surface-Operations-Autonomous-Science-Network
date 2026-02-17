from pathlib import Path
import sys

import pytest

from _helpers import FakeBus, load_module

SRC_ROOT = Path(__file__).resolve().parents[1]
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from rover_core.task_model import compute_fault_probability
from rover_core.task_model import compute_task_duration_steps
from rover_core.task_model import load_catalog
from rover_core.task_model import normalize_task_request
from rover_core.task_model import select_best_rover


@pytest.fixture()
def task_catalog():
    return load_catalog()


@pytest.fixture()
def earth_node(monkeypatch):
    bus = FakeBus()
    earth_module = load_module(monkeypatch, 'rover_core.earth_node', bus)
    return earth_module.EarthNode()


def test_catalog_validates_required_base_rates(task_catalog):
    assert task_catalog['difficulty_levels']['L1']['base_fault_rate'] == 0.01
    assert task_catalog['difficulty_levels']['L2']['base_fault_rate'] == 0.03
    assert task_catalog['difficulty_levels']['L3']['base_fault_rate'] == 0.06
    assert task_catalog['difficulty_levels']['L4']['base_fault_rate'] == 0.10
    assert task_catalog['difficulty_levels']['L5']['base_fault_rate'] == 0.18


def test_difficulty_l1_vs_l5_changes_duration_and_risk(task_catalog):
    context = {
        'terrain_difficulty': 0.4,
        'comm_quality': 0.8,
        'battery': 0.7,
        'solar_intensity': 0.7,
        'thermal_stress': 0.3,
        'lunar_time_state': 'DAYLIGHT',
        'capability_match': True,
    }

    movement_l1 = normalize_task_request(
        task_catalog,
        task_id='MOVE-L1',
        task_type='movement',
        difficulty_level='L1',
    )
    movement_l5 = normalize_task_request(
        task_catalog,
        task_id='MOVE-L5',
        task_type='movement',
        difficulty_level='L5',
    )

    l1_steps = compute_task_duration_steps(task_catalog, 'movement', 'L1', context)
    l5_steps = compute_task_duration_steps(task_catalog, 'movement', 'L5', context)

    l1_risk, _ = compute_fault_probability(task_catalog, 'movement', 'L1', context)
    l5_risk, _ = compute_fault_probability(task_catalog, 'movement', 'L5', context)

    assert movement_l1['difficulty_level'] == 'L1'
    assert movement_l5['difficulty_level'] == 'L5'
    assert l5_steps > l1_steps
    assert l5_risk > l1_risk


def test_low_battery_and_lunar_night_increase_predicted_risk(task_catalog):
    day_context = {
        'battery': 0.9,
        'solar_intensity': 0.85,
        'terrain_difficulty': 0.25,
        'comm_quality': 0.9,
        'thermal_stress': 0.2,
        'lunar_time_state': 'DAYLIGHT',
        'capability_match': True,
    }
    night_context = {
        'battery': 0.18,
        'solar_intensity': 0.1,
        'terrain_difficulty': 0.65,
        'comm_quality': 0.55,
        'thermal_stress': 0.72,
        'lunar_time_state': 'NIGHT',
        'capability_match': True,
    }

    day_risk, _ = compute_fault_probability(task_catalog, 'science', 'L3', day_context)
    night_risk, _ = compute_fault_probability(
        task_catalog,
        'science',
        'L3',
        night_context,
    )

    assert night_risk > day_risk


def test_assignment_changes_with_task_capability_needs(task_catalog):
    fleet = {
        'rover_1': {
            'state': 'IDLE',
            'battery': 0.95,
            'solar_exposure': 0.8,
            'terrain_difficulty': 0.25,
            'comm_quality': 0.88,
            'thermal_stress': 0.2,
            'position': {'lat': -43.3, 'lon': -11.2},
        },
        'rover_2': {
            'state': 'IDLE',
            'battery': 0.82,
            'solar_exposure': 0.7,
            'terrain_difficulty': 0.35,
            'comm_quality': 0.82,
            'thermal_stress': 0.25,
            'position': {'lat': -43.2, 'lon': -11.15},
        },
        'rover_3': {
            'state': 'IDLE',
            'battery': 0.76,
            'solar_exposure': 0.65,
            'terrain_difficulty': 0.38,
            'comm_quality': 0.8,
            'thermal_stress': 0.28,
            'position': {'lat': -43.4, 'lon': -11.3},
        },
    }

    science_task = normalize_task_request(
        task_catalog,
        task_id='SCI-01',
        task_type='science',
        difficulty_level='L2',
    )
    digging_task = normalize_task_request(
        task_catalog,
        task_id='DIG-01',
        task_type='digging',
        difficulty_level='L2',
    )

    science_selection = select_best_rover(task_catalog, fleet, science_task)
    digging_selection = select_best_rover(task_catalog, fleet, digging_task)

    assert science_selection['selected_rover'] in {'rover_1', 'rover_2'}
    assert digging_selection['selected_rover'] == 'rover_3'


def test_rejection_when_no_feasible_rover(task_catalog):
    fleet = {
        'rover_1': {
            'state': 'EXECUTING',
            'battery': 0.14,
            'solar_exposure': 0.1,
            'terrain_difficulty': 0.75,
            'comm_quality': 0.5,
            'thermal_stress': 0.8,
            'position': {'lat': -43.3, 'lon': -11.2},
        },
        'rover_2': {
            'state': 'SAFE_MODE',
            'battery': 0.2,
            'solar_exposure': 0.15,
            'terrain_difficulty': 0.8,
            'comm_quality': 0.45,
            'thermal_stress': 0.85,
            'position': {'lat': -43.1, 'lon': -11.1},
        },
    }
    hard_task = normalize_task_request(
        task_catalog,
        task_id='SAMPLE-FAIL',
        task_type='sample-handling',
        difficulty_level='L5',
    )

    selection = select_best_rover(task_catalog, fleet, hard_task)
    assert selection['selected_rover'] is None
    assert selection['reject_reason']


def test_earth_auto_assign_legacy_call_still_works(earth_node, monkeypatch):
    earth_node.fleet_registry = {
        'rover_1': {
            'state': 'IDLE',
            'battery': 0.8,
            'solar_exposure': 0.7,
            'terrain_difficulty': 0.3,
            'comm_quality': 0.82,
            'thermal_stress': 0.22,
            'position': {'lat': -43.3, 'lon': -11.2},
        },
        'rover_2': {
            'state': 'IDLE',
            'battery': 0.72,
            'solar_exposure': 0.62,
            'terrain_difficulty': 0.4,
            'comm_quality': 0.78,
            'thermal_stress': 0.3,
            'position': {'lat': -43.4, 'lon': -11.3},
        },
    }

    dispatched = []

    def record_send(rover_id, cmd_type, task_id=None, task_request=None, assignment_context=None):
        dispatched.append((rover_id, cmd_type, task_id, task_request, assignment_context))

    monkeypatch.setattr(earth_node, 'send_command', record_send)

    assert earth_node.auto_assign_task('legacy-task-id') is True
    assert len(dispatched) == 1
    rover_id, cmd_type, task_id, task_request, assignment_context = dispatched[0]
    assert cmd_type == 'START_TASK'
    assert task_id == 'legacy-task-id'
    assert task_request['task_type'] == 'movement'
    assert task_request['difficulty_level'] == 'L2'
    assert rover_id in {'rover_1', 'rover_2'}
    assert assignment_context['selected_rover'] == rover_id
