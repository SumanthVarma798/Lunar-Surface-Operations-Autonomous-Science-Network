# Issues #12 and #13 Walkthrough

## Implemented Scope
- Added assignment-focused unit tests for Earth rover selection and dispatch logic.
- Added an integration-style multi-rover command-flow test covering auto assignment, rover state transitions, and ACK routing.

## Files Added
- `lunar_ops/rover_ws/src/rover_core/test/_helpers.py`
- `lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py`
- `lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py`

## Issue #12: Unit tests for task assignment algorithm

Implemented coverage:
- Highest-battery selection among multiple `IDLE` rovers.
- Rejection when all rovers are unavailable (`EXECUTING` / `SAFE_MODE`).
- Solar exposure influence in scoring behavior.
- Manual assignment path dispatches explicitly targeted rover (manual override behavior).
- Edge cases:
  - Single-rover assignment.
  - All-low-battery fleet selection still resolves deterministically.
- Added a focused line-coverage guard test for assignment functions (`select_best_rover`, `auto_assign_task`, `assign_task`) with threshold `>= 80%`.

## Issue #13: Integration test for multi-rover command flow

Implemented scenario:
1. Initialized Earth + 3 rover test doubles with a simulated space-link topic router.
2. Published initial telemetry for all three rovers.
3. Sent first `START_TASK` via auto-assignment and verified:
   - Exactly one rover transitions to `EXECUTING`.
   - ACK reaches Earth path with matching `rover_id`.
4. Sent second task and verified:
   - Command targets a different rover than first assignment.
5. Ensured clean command tracking (`pending_commands` cleared) during test flow.

## Verification Performed

### Main test run
```bash
python3 -m pytest -v lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py
```

Result:
- 8 passed, 0 failed.

### Coverage check
```bash
python3 -m pytest -v --cov=rover_core.earth_node --cov-report=term-missing lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py
```

Notes:
- Whole-file Earth node coverage is lower because assignment logic is only a subset of the node.
- Assignment-logic-specific threshold is enforced by:
  - `test_assignment_logic_line_coverage_above_80_percent`

## Acceptance Criteria Check

### Issue #12
- [x] All test cases pass
- [x] Coverage > 80% for assignment logic

### Issue #13
- [x] Test launches full fleet (simulated Earth + Space Link router + 3 rovers)
- [x] Verifies correct rover selection
- [x] Checks ACK routing
- [x] Clean teardown/command state cleanup
