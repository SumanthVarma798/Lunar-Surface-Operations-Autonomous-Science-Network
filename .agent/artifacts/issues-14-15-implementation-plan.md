# Issues #14 and #15 Implementation Plan

## Scope
- Issue #14: `Manual testing scenarios for web dashboard`
- Issue #15: `Update README with multi-rover instructions`

## Issue Requirements Extracted

### Issue #14
- Create testing results documentation:
  - `docs/phase1_testing_results.md` (new file)
- Execute and document scenarios:
  1. Basic fleet operations (auto task assignment)
  2. Manual rover selection
  3. Safe mode handling across fleet
  4. Battery-based selection verification
  5. Telemetry load test (5 rovers at 0.5 Hz)
- Acceptance criteria:
  - All scenarios tested
  - Results documented with screenshots
  - Bugs filed for any issues found

### Issue #15
- Update:
  - `README.md`
- Required updates:
  - Fleet architecture diagram
  - Launch instructions
  - Fleet command usage
  - Screenshots
- Acceptance criteria:
  - README accurately describes fleet system
  - Commands work as documented
  - Screenshots reflect current UI

## Implementation Approach

### 1) Gather runtime evidence from the current web simulator
- Start the web simulator locally from `web-sim` on `http://localhost:8080`.
- Validate fleet-specific UI elements and command routing behavior against the five testing scenarios.
- Capture fresh dashboard screenshots that show the multi-rover fleet UI and command interactions.

### 2) Produce Phase 1 testing report (`docs/phase1_testing_results.md`)
- Add a scenario-by-scenario section with:
  - Setup and parameters used
  - Steps performed
  - Observed behavior/results
  - Pass/fail outcome
  - Linked screenshot evidence
- Include a bug log section:
  - If issues are found, list issue IDs and symptoms.
  - If no issues are found, explicitly state no bug tickets were needed.

### 3) Update README (`README.md`) for fleet operations
- Replace single-rover framing with fleet-focused descriptions where outdated.
- Update architecture diagram to show fleet-aware command/telemetry flow.
- Update launch/usage instructions to reflect current `web-sim` workflow.
- Add fleet command routing documentation:
  - Auto assignment mode
  - Manual rover targeting
  - Safe mode and reset behavior
- Refresh screenshot section to use current fleet UI captures.

### 4) Verification
- Functional verification:
  - Serve `web-sim` and ensure README commands run exactly as documented.
  - Confirm UI contains fleet banner, fleet grid, rover target selector, and command/ACK counters.
- Documentation verification:
  - Validate all five issue-14 scenarios are covered in the new report.
  - Validate README sections requested by issue #15 are present and accurate.

## Planned Files
- New: `docs/phase1_testing_results.md`
- Update: `README.md`
- New (post-implementation): `.agent/artifacts/issues-14-15-walkthrough.md`

## Acceptance Checklist

### Issue #14
- [ ] Basic fleet operations scenario executed and documented
- [ ] Manual rover selection scenario executed and documented
- [ ] Safe mode handling scenario executed and documented
- [ ] Battery-based selection scenario executed and documented
- [ ] Telemetry load test scenario executed and documented
- [ ] Screenshot evidence included for results
- [ ] Bug section included (with filed issues if applicable)

### Issue #15
- [ ] README architecture diagram updated for fleet model
- [ ] README launch instructions updated and verified
- [ ] Fleet command usage documented
- [ ] Screenshot section updated with current fleet UI
- [ ] Documented commands validated in local run
