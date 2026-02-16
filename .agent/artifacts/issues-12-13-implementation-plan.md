# Issues #12 and #13 Implementation Plan

## Scope
- Issue #12: `Write unit tests for task assignment algorithm`
- Issue #13: `Integration test: Multi-rover command flow`

## Issue Requirements Extracted

### Issue #12
- Add new unit test file:
  - `lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py`
- Required test coverage:
  - Selects rover with highest battery when multiple `IDLE`
  - Rejects assignment when all rovers are `EXECUTING` or `SAFE_MODE`
  - Includes solar exposure influence in scoring
  - Manual assignment overrides auto
  - Edge cases (single rover, all low battery)
- Acceptance criteria:
  - All test cases pass
  - Coverage > 80% for assignment logic

### Issue #13
- Add new integration test file:
  - `lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py`
- Required scenario:
  1. Launch 3 rovers + space link + earth
  2. Send `START_TASK` with auto-assignment
  3. Verify exactly 1 rover transitions to `EXECUTING`
  4. Verify ACK received with correct `rover_id`
  5. Send second task and verify assignment targets a different rover
- Acceptance criteria:
  - Test launches full fleet
  - Verifies correct rover selection
  - Checks ACK routing
  - Clean teardown

## Implementation Approach

### 1) Shared test harness strategy
- Use pytest with lightweight fake ROS modules injected into `sys.modules` (`rclpy`, `rclpy.node`, `std_msgs.msg`) so tests can import `EarthNode` in CI without requiring a full ROS runtime.
- Instantiate `EarthNode` instances via `EarthNode.__new__(EarthNode)` for pure logic tests and manually seed only fields needed by assignment paths (`fleet_registry`, `_lock`, publishers, pending commands, etc.).
- For integration scenario, create a deterministic in-process fleet harness (3 rover states + Earth command path + ACK callback events) that validates command routing and state transitions end-to-end at the application layer.

### 2) `test_task_assignment.py` (Issue #12)
- Add focused unit tests for:
  - `select_best_rover` battery precedence with multiple `IDLE` rovers.
  - Solar bonus impact (`solar_exposure >= 0.5` vs `< 0.5`) in score ordering.
  - `auto_assign_task` rejection when no `IDLE` rover is available.
  - `auto_assign_task` success path calls `send_command(<best>, "START_TASK", task_id)`.
  - `assign_task` manual override sends command directly to requested rover.
  - Edge cases:
    - single rover available
    - all low battery but `IDLE` (still selects deterministic best score)

### 3) `test_fleet_integration.py` (Issue #13)
- Build a multi-rover flow test that:
  - Initializes Earth-side fleet registry with 3 rovers.
  - Simulates telemetry updates for each rover.
  - Executes first `auto_assign_task` and captures targeted rover command.
  - Simulates rover state transition + ACK callback for selected rover.
  - Executes second `auto_assign_task` after first rover is marked `EXECUTING`.
  - Asserts second assignment selects a different rover.
  - Verifies ACK handling removes matching pending command with correct `rover_id`.
  - Includes deterministic cleanup fixture for mocked module state and any mutable globals.

### 4) Verification steps
- Run:
  - `python3 -m pytest -v lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py`
- Coverage check for assignment logic (if `pytest-cov` is available):
  - `python3 -m pytest -v --cov=lunar_ops/rover_ws/src/rover_core/rover_core/earth_node.py --cov-report=term-missing lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py`
- If `pytest-cov` is unavailable locally, still run full tests and document coverage-tool gap in walkthrough artifact.

## Planned Files
- New: `lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py`
- New: `lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py`
- New: `.agent/artifacts/issues-12-13-walkthrough.md` (after implementation/verification)

## Acceptance Checklist

### Issue #12
- [ ] Highest battery `IDLE` rover selected
- [ ] Reject when all rovers unavailable (`EXECUTING` / `SAFE_MODE`)
- [ ] Solar exposure affects scoring outcome
- [ ] Manual assignment path overrides auto-selection
- [ ] Single-rover and low-battery edge cases covered
- [ ] Tests pass
- [ ] Coverage target validated for assignment logic

### Issue #13
- [ ] 3-rover flow initialized in integration test harness
- [ ] First auto task transitions exactly one rover to `EXECUTING`
- [ ] ACK routed/processed with correct `rover_id`
- [ ] Second task assigned to different rover
- [ ] Integration test teardown is clean/deterministic
