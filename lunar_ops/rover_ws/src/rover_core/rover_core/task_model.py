import json
import math
import re
from pathlib import Path

FALLBACK_TASK_TYPE = 'movement'
FALLBACK_DIFFICULTY = 'L2'
FALLBACK_MISSION_PHASE = 'CY3-ops'


def _clamp(value, low=0.0, high=1.0):
    return max(low, min(high, value))


def _as_float(value, fallback=0.0):
    try:
        return float(value)
    except (TypeError, ValueError):
        return fallback


def _load_json(path):
    with path.open('r', encoding='utf-8') as fp:
        return json.load(fp)


def find_catalog_path(explicit_path=None):
    if explicit_path:
        candidate = Path(explicit_path).expanduser().resolve()
        if candidate.exists():
            return candidate

    local_catalog = Path(__file__).with_name('chandrayaan_task_catalog.json')
    if local_catalog.exists():
        return local_catalog

    current = Path(__file__).resolve()
    for parent in current.parents:
        candidate = parent / 'web-sim' / 'assets' / 'chandrayaan_task_catalog.json'
        if candidate.exists():
            return candidate

    raise FileNotFoundError('Cannot locate chandrayaan_task_catalog.json')


def validate_catalog(catalog):
    if 'difficulty_levels' not in catalog or 'task_types' not in catalog:
        raise ValueError('Task catalog missing required keys')

    required_levels = ['L1', 'L2', 'L3', 'L4', 'L5']
    for level in required_levels:
        if level not in catalog['difficulty_levels']:
            raise ValueError(f'Missing difficulty level: {level}')

    required_level_rates = {
        'L1': 0.01,
        'L2': 0.03,
        'L3': 0.06,
        'L4': 0.10,
        'L5': 0.18,
    }
    for level, expected_rate in required_level_rates.items():
        actual = float(catalog['difficulty_levels'][level]['base_fault_rate'])
        if abs(actual - expected_rate) > 1e-9:
            raise ValueError(
                f'Unexpected base fault rate for {level}: {actual} (expected {expected_rate})'
            )

    for task_type, task_cfg in catalog['task_types'].items():
        for key in ['required_capabilities', 'base_steps', 'mission_phase']:
            if key not in task_cfg:
                raise ValueError(f'Task type {task_type} missing key: {key}')


def load_catalog(explicit_path=None):
    catalog_path = find_catalog_path(explicit_path)
    catalog = _load_json(catalog_path)
    validate_catalog(catalog)
    return catalog


def get_rover_capabilities(rover_id):
    rover_str = str(rover_id or '')
    match = re.search(r'(\d+)$', rover_str)
    rover_index = int(match.group(1)) if match else 1

    profile_by_mod = {
        1: {'mobility', 'imaging', 'science'},
        2: {'mobility', 'science', 'sample-logistics', 'imaging'},
        0: {'mobility', 'excavation', 'manipulation', 'sample-logistics', 'imaging'},
    }

    capabilities = set(profile_by_mod.get(rover_index % 3, {'mobility', 'imaging'}))
    capabilities.add('mobility')
    return capabilities


def normalize_task_request(catalog, task_id, task_type=None, difficulty_level=None,
                           mission_phase=None, required_capabilities=None,
                           target_site=None):
    task_type_key = str(task_type or FALLBACK_TASK_TYPE).strip().lower()
    if task_type_key not in catalog['task_types']:
        task_type_key = FALLBACK_TASK_TYPE

    diff_key = str(difficulty_level or FALLBACK_DIFFICULTY).strip().upper()
    if diff_key not in catalog['difficulty_levels']:
        diff_key = FALLBACK_DIFFICULTY

    task_cfg = catalog['task_types'][task_type_key]

    if required_capabilities:
        req_caps = [
            str(cap).strip().lower()
            for cap in required_capabilities
            if str(cap).strip()
        ]
    else:
        req_caps = list(task_cfg.get('required_capabilities', []))

    normalized_task_id = str(task_id or f'{task_type_key}-{diff_key}').strip()
    if not normalized_task_id:
        normalized_task_id = f'{task_type_key}-{diff_key}'

    return {
        'task_id': normalized_task_id,
        'task_type': task_type_key,
        'difficulty_level': diff_key,
        'mission_phase': mission_phase or task_cfg.get('mission_phase', FALLBACK_MISSION_PHASE),
        'required_capabilities': req_caps,
        'target_site': target_site,
    }


def get_lunar_time_state(solar_intensity):
    solar = _clamp(_as_float(solar_intensity, 0.5))
    if solar >= 0.55:
        return 'DAYLIGHT'
    if solar <= 0.30:
        return 'NIGHT'
    return 'TERMINATOR'


def compute_task_duration_steps(catalog, task_type, difficulty_level, context):
    task_cfg = catalog['task_types'][task_type]
    diff_cfg = catalog['difficulty_levels'][difficulty_level]

    base_steps = int(task_cfg['base_steps'])
    difficulty_multiplier = _as_float(diff_cfg.get('duration_multiplier', 1.0), 1.0)
    terrain_difficulty = _clamp(_as_float(context.get('terrain_difficulty', 0.3), 0.3))
    comm_quality = _clamp(_as_float(context.get('comm_quality', 0.8), 0.8))

    terrain_factor = 1.0 + terrain_difficulty * _as_float(
        task_cfg.get('terrain_duration_sensitivity', 0.3), 0.3
    )
    comm_factor = 1.0 + (1.0 - comm_quality) * 0.2

    steps = int(round(base_steps * difficulty_multiplier * terrain_factor * comm_factor))
    steps = max(int(task_cfg.get('min_steps', 3)), steps)
    steps = min(int(task_cfg.get('max_steps', 60)), steps)
    return steps


def compute_fault_probability(catalog, task_type, difficulty_level, context):
    task_cfg = catalog['task_types'][task_type]
    diff_cfg = catalog['difficulty_levels'][difficulty_level]

    base_risk = _as_float(diff_cfg.get('base_fault_rate', 0.03), 0.03)

    battery = _clamp(_as_float(context.get('battery', 1.0), 1.0))
    solar = _clamp(_as_float(context.get('solar_intensity', context.get('solar_exposure', 0.5)), 0.5))
    terrain = _clamp(_as_float(context.get('terrain_difficulty', 0.3), 0.3))
    comm_quality = _clamp(_as_float(context.get('comm_quality', 0.8), 0.8))
    thermal_stress = _clamp(_as_float(context.get('thermal_stress', 0.3), 0.3))
    capability_match = bool(context.get('capability_match', True))

    lunar_state = context.get('lunar_time_state') or get_lunar_time_state(solar)

    battery_penalty = max(0.0, 0.45 - battery) * _as_float(
        task_cfg.get('battery_sensitivity', 0.8), 0.8
    ) * 0.22
    solar_penalty = max(0.0, 0.40 - solar) * _as_float(
        task_cfg.get('solar_sensitivity', 0.6), 0.6
    ) * 0.18
    terrain_penalty = terrain * _as_float(task_cfg.get('terrain_sensitivity', 0.6), 0.6) * 0.12
    comm_penalty = (1.0 - comm_quality) * _as_float(
        task_cfg.get('comm_sensitivity', 0.5), 0.5
    ) * 0.10
    thermal_penalty = thermal_stress * _as_float(
        task_cfg.get('thermal_sensitivity', 0.6), 0.6
    ) * 0.10

    lunar_penalty = {
        'DAYLIGHT': 0.0,
        'TERMINATOR': 0.02,
        'NIGHT': 0.06,
    }.get(str(lunar_state).upper(), 0.02)

    capability_penalty = 0.0 if capability_match else 0.12

    battery_bonus = max(0.0, battery - 0.80) * 0.03
    solar_bonus = max(0.0, solar - 0.85) * 0.02

    risk = (
        base_risk
        + battery_penalty
        + solar_penalty
        + terrain_penalty
        + comm_penalty
        + thermal_penalty
        + lunar_penalty
        + capability_penalty
        - battery_bonus
        - solar_bonus
    )

    risk = max(_as_float(task_cfg.get('risk_floor', 0.0), 0.0), risk)
    risk = _clamp(risk, 0.0, 0.6)

    return risk, {
        'base_risk': round(base_risk, 4),
        'battery_penalty': round(battery_penalty, 4),
        'solar_penalty': round(solar_penalty, 4),
        'terrain_penalty': round(terrain_penalty, 4),
        'comm_penalty': round(comm_penalty, 4),
        'thermal_penalty': round(thermal_penalty, 4),
        'lunar_penalty': round(lunar_penalty, 4),
        'capability_penalty': round(capability_penalty, 4),
        'battery_bonus': round(battery_bonus, 4),
        'solar_bonus': round(solar_bonus, 4),
        'lunar_time_state': str(lunar_state).upper(),
    }


def _distance_margin(target_site, position):
    if not target_site or not position:
        return 0.75

    try:
        dlat = float(target_site.get('lat', 0.0)) - float(position.get('lat', 0.0))
        dlon = float(target_site.get('lon', 0.0)) - float(position.get('lon', 0.0))
    except (TypeError, ValueError):
        return 0.6

    distance = math.sqrt(dlat ** 2 + dlon ** 2)
    # scale around a 2-degree mission envelope
    normalized = _clamp(1.0 - min(distance / 2.0, 1.0), 0.0, 1.0)
    return normalized


def score_rover_for_task(catalog, rover_id, status, task_request):
    state = str(status.get('state', 'UNKNOWN')).upper()
    battery = _clamp(_as_float(status.get('battery', 0.0), 0.0))
    solar = _clamp(_as_float(status.get('solar_exposure', status.get('solar_intensity', 0.5)), 0.5))

    terrain_difficulty = _clamp(_as_float(status.get('terrain_difficulty', 0.3), 0.3))
    comm_quality = _clamp(_as_float(status.get('comm_quality', 0.8), 0.8))
    thermal_stress = _clamp(_as_float(status.get('thermal_stress', 0.3), 0.3))

    capabilities = get_rover_capabilities(rover_id)
    required_caps = set(task_request.get('required_capabilities', []))
    capability_match = required_caps.issubset(capabilities)

    context = {
        'battery': battery,
        'solar_intensity': solar,
        'terrain_difficulty': terrain_difficulty,
        'comm_quality': comm_quality,
        'thermal_stress': thermal_stress,
        'capability_match': capability_match,
    }
    predicted_risk, risk_breakdown = compute_fault_probability(
        catalog,
        task_request['task_type'],
        task_request['difficulty_level'],
        context,
    )

    distance_margin = _distance_margin(task_request.get('target_site'), status.get('position'))
    accessibility_margin = 1.0 - terrain_difficulty

    score_breakdown = {
        'capability_match': 1.0 if capability_match else 0.0,
        'battery_margin': battery,
        'solar_margin': solar,
        'thermal_margin': 1.0 - thermal_stress,
        'comm_margin': comm_quality,
        'distance_margin': distance_margin,
        'accessibility_margin': accessibility_margin,
        'risk_margin': 1.0 - (predicted_risk / 0.6),
    }

    weighted_score = (
        score_breakdown['capability_match'] * 0.28
        + score_breakdown['battery_margin'] * 0.20
        + score_breakdown['solar_margin'] * 0.12
        + score_breakdown['thermal_margin'] * 0.10
        + score_breakdown['comm_margin'] * 0.10
        + ((score_breakdown['distance_margin'] + score_breakdown['accessibility_margin']) / 2.0) * 0.10
        + score_breakdown['risk_margin'] * 0.10
    )

    difficulty_cfg = catalog['difficulty_levels'][task_request['difficulty_level']]
    min_battery = _as_float(difficulty_cfg.get('min_battery', 0.0), 0.0)

    reject_reasons = []
    if state != 'IDLE':
        reject_reasons.append(f'state={state}')
    if battery < min_battery:
        reject_reasons.append(f'battery<{min_battery:.2f}')
    if not capability_match:
        reject_reasons.append('capability_mismatch')
    if predicted_risk > 0.45:
        reject_reasons.append('predicted_risk_too_high')

    feasible = not reject_reasons

    return {
        'rover_id': rover_id,
        'feasible': feasible,
        'reject_reasons': reject_reasons,
        'score': round(weighted_score, 4),
        'score_breakdown': {k: round(v, 4) for k, v in score_breakdown.items()},
        'predicted_fault_probability': round(predicted_risk, 4),
        'risk_breakdown': risk_breakdown,
        'capabilities': sorted(capabilities),
    }


def select_best_rover(catalog, fleet_registry, task_request):
    scored = []
    for rover_id, status in fleet_registry.items():
        scored.append(score_rover_for_task(catalog, rover_id, status, task_request))

    feasible = [item for item in scored if item['feasible']]
    if not feasible:
        scored_sorted = sorted(scored, key=lambda item: item['score'], reverse=True)
        top = scored_sorted[0] if scored_sorted else None
        reject_reason = 'no_rover_passed_feasibility_threshold'
        if top and top['reject_reasons']:
            reject_reason = ','.join(top['reject_reasons'])
        return {
            'selected_rover': None,
            'reject_reason': reject_reason,
            'scored_candidates': scored_sorted,
        }

    feasible_sorted = sorted(feasible, key=lambda item: item['score'], reverse=True)
    return {
        'selected_rover': feasible_sorted[0]['rover_id'],
        'reject_reason': None,
        'scored_candidates': feasible_sorted,
    }
