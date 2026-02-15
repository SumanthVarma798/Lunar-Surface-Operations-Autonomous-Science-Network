# Issue #7 Walkthrough: ROS 2 Fleet Launch File

## Implemented
- Added `lunar_ops/rover_ws/src/rover_core/launch/fleet_launch.py`.
- Added launch-install packaging in `lunar_ops/rover_ws/src/rover_core/setup.py`.

## Launch Behavior
- New single command entry point:
  - `ros2 launch rover_core fleet_launch.py`
- Starts:
  - `space_link_node`
  - `fleet_manager`
  - `earth_node`
  - `telemetry_monitor`
  - `rover_node` instances for `rover_1..rover_N`
- Added launch argument:
  - `rover_count` (default `3`)

## Parameter Wiring
- Each rover receives `rover_id=<rover_n>`.
- Earth station receives `rover_ids=<comma-separated rover ids>`.
- Rover IDs are generated from `rover_count`.

## Namespacing
- Shared nodes:
  - `/space_link`
  - `/fleet`
  - `/earth`
- Rover nodes:
  - `/rover/rover_1`, `/rover/rover_2`, ... `/rover/rover_N`

## Verification
- Syntax checks:
  - `PYTHONPYCACHEPREFIX=/tmp/codex-pyc python3 -m py_compile lunar_ops/rover_ws/src/rover_core/setup.py lunar_ops/rover_ws/src/rover_core/launch/fleet_launch.py`
- Acceptance criteria checklist:
  - Single command starts entire fleet: Implemented via `fleet_launch.py`.
  - All nodes properly namespaced: Implemented for shared + per-rover nodes.
  - Parameters passed correctly: `rover_id` and `rover_ids` wired from launch config.
  - Configurable rover count: Implemented via `rover_count`.

## Screenshots / Recordings
- None captured for this issue (ROS launch UI not applicable).
