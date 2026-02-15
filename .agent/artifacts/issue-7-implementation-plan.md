# Issue #7 Implementation Plan: Create ROS 2 launch file for fleet

## Issue Summary
- Title: Create ROS 2 launch file for fleet
- Goal: Add one launch entry point that starts the full multi-rover stack.
- Required new file: `lunar_ops/rover_ws/src/rover_core/launch/fleet_launch.py`

## Acceptance Criteria Mapping
1. Single command starts entire fleet
- Implement `fleet_launch.py` so `ros2 launch rover_core fleet_launch.py` starts:
  - Space link node
  - Rover nodes (default 3 instances)
  - Fleet manager
  - Earth station
  - Telemetry monitor

2. All nodes properly namespaced
- Launch each rover in a dedicated rover namespace (`/rover/rover_1`, etc.) with unique node names.
- Launch shared services in stable namespaces (`/space_link`, `/fleet`, `/earth`) with explicit node names.

3. Parameters passed correctly
- Pass `rover_id` for each rover node.
- Pass `rover_ids` (comma-separated list) to earth and space link nodes.
- Keep defaults aligned with the launched rover set.

4. Configurable rover count
- Add launch argument `rover_count` (default `3`).
- Dynamically generate rover node actions for `rover_1..rover_N`.

## Implementation Tasks
1. Create `fleet_launch.py`
- Define `DeclareLaunchArgument` for rover count.
- Build rover ID list from launch config via `OpaqueFunction`.
- Instantiate all required nodes and return assembled launch actions.

2. Update package installation metadata (required for `ros2 launch`)
- Update `lunar_ops/rover_ws/src/rover_core/setup.py` to install `launch/*.py` into `share/rover_core/launch`.

3. Verify
- Syntax-check Python sources.
- Validate acceptance checklist against launch content.

## Notes / Assumptions
- Issue text uses rover names like `rover-1`; ROS naming conventions are safer with underscores, so this plan uses `rover_1..rover_N`.
- Existing node code includes absolute topic paths; namespaces will apply to node identity and process organization, while rover-specific routing still relies on `rover_id`.
