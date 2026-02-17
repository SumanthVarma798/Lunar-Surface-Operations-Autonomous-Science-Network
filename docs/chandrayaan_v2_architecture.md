# Chandrayaan v2 Architecture

## Objective

Replace generic fleet tasking with a context-aware rover-task model aligned to Chandrayaan-inspired mission operations.

## Core Components

1. `task_model.py`
- Loads and validates TaskCatalog.
- Normalizes task requests.
- Computes task duration and predicted fault probability.
- Scores rover feasibility and assignment ranking.

2. `earth_node.py`
- Accepts structured task requests (`task_type`, `difficulty_level`, mission metadata).
- Runs explainable assignment scoring.
- Dispatches `START_TASK` payloads with `assignment_score_breakdown` and predicted risk.

3. `rover_node.py`
- Executes variable-duration tasks based on catalog + context.
- Updates risk dynamically each step.
- Publishes expanded telemetry context.

4. `web-sim/simulation.js`
- Browser parity model of task and assignment logic.
- Uses same task taxonomy/difficulty model as ROS path.

## Task Families

- `movement`
- `science`
- `digging`
- `pushing`
- `photo`
- `sample-handling`

## Difficulty and Risk

Base fault rates:

- `L1=1%`
- `L2=3%`
- `L3=6%`
- `L4=10%`
- `L5=18%`

Dynamic modifiers:

- battery SOC
- lunar day/night state
- solar intensity
- terrain difficulty
- comm quality
- thermal stress
- capability match

Final risk clamp: `0..0.6`.

## Assignment Output Contract

- `selected_rover`
- `reject_reason`
- `score_breakdown`
- `predicted_fault_probability`

## Telemetry Additions

- `active_task_type`
- `active_task_difficulty`
- `task_total_steps`
- `predicted_fault_probability`
- `assignment_score_breakdown`
- `lunar_time_state`
- `solar_intensity`
- `terrain_difficulty`
- `comm_quality`
- `thermal_stress`
