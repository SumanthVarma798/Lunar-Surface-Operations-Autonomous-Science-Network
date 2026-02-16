# Phase 1 Fleet Dashboard Manual Testing Results

Date: 2026-02-16  
Scope: Issue #14 manual scenario validation for `web-sim` fleet dashboard.

## Test Environment

- Runtime: `python3 -m http.server 8080 --directory web-sim`
- Access URL: `http://127.0.0.1:8080/index.html`
- Browser capture mode: headless Chrome (deterministic replay URLs)
- Baseline transport settings for deterministic checks:
  - `latency=0.2`
  - `jitter=0`
  - `drop=0`
  - `fault=0`

## Scenario 1: Basic Fleet Operations (Auto Task Assignment)

- Objective: verify auto-selection dispatches a task and transitions one rover to `EXECUTING`.
- Steps:
  1. Open `?scenario=basic-auto&task=AUTO-001&delay_ms=18000`.
  2. Wait for command dispatch and ACK.
  3. Confirm one rover enters `EXECUTING` and task progress increments.
- Result: PASS
- Observed:
  - Fleet banner updates to `2 IDLE · 1 EXECUTING · 0 SAFE_MODE`.
  - Command/ACK counters increment to `1 / 1`.
  - Fleet card and topology both show the same rover executing.
- Evidence: `docs/screenshots/dashboard-executing.png`

## Scenario 2: Manual Rover Selection

- Objective: verify manual target selection routes command to the selected rover.
- Steps:
  1. Open `?scenario=manual-select&manual_rover=rover-2&task=MANUAL-002&delay_ms=18000`.
  2. Confirm target selector switches to `Rover-2`.
  3. Confirm `Rover-2` receives and executes the task.
- Result: PASS
- Observed:
  - Target control displays `Rover-2`.
  - `Rover-2` card transitions to `EXECUTING`.
  - Telemetry stream logs execution steps for `[rover-2]`.
- Evidence: `docs/screenshots/phase1-manual-selection.png`

## Scenario 3: Safe Mode Handling Across Fleet

- Objective: verify safe-mode command transitions active rover to `SAFE_MODE`.
- Steps:
  1. Open `?scenario=safe-mode&task=SAFE-001&delay_ms=15000`.
  2. Let task start, then scenario sends `GO_SAFE`.
  3. Confirm state transition and fleet distribution update.
- Result: PASS
- Observed:
  - Fleet banner updates to `2 IDLE · 0 EXECUTING · 1 SAFE_MODE`.
  - Target rover state shows `SAFE_MODE` on card and topology.
  - Logs include `Commanded to SAFE_MODE`.
- Evidence: `docs/screenshots/dashboard-safe-mode.png`

## Scenario 4: Battery-Based Selection Verification

- Objective: verify auto assignment prioritizes rover with highest battery under idle conditions.
- Steps:
  1. Open  
     `?scenario=battery-select&task=BATT-001&delay_ms=18000&battery=rover-1:0.22,rover-2:0.88,rover-3:0.41`.
  2. Keep target in `Auto-Select (Best Rover)`.
  3. Confirm selected rover is highest-battery candidate.
- Result: PASS
- Observed:
  - Pre-dispatch fleet batteries: rover-1 `22%`, rover-2 `88%`, rover-3 `41%`.
  - `Rover-2` is auto-selected and transitions to `EXECUTING`.
  - Lower-battery rovers remain `IDLE`.
- Evidence: `docs/screenshots/phase1-battery-selection.png`

## Scenario 5: Telemetry Load Test (5 Rovers @ 0.5 Hz)

- Objective: verify dashboard stability and telemetry rendering with five active rover streams.
- Steps:
  1. Open `?rovers=5`.
  2. Allow telemetry to stream for one capture interval.
  3. Confirm fleet totals and feed show all rover IDs.
- Result: PASS
- Observed:
  - Fleet banner shows `Total Rovers: 5`.
  - State distribution shows `5 IDLE`.
  - Telemetry feed shows entries for rover-1 through rover-5.
  - With rover telemetry interval at 2 seconds, per-rover telemetry rate is 0.5 Hz.
- Evidence: `docs/screenshots/phase1-telemetry-load.png`

## Defect Summary

- No functional regressions or defects were observed in the five required scenarios.
- Bug tickets filed: none (no issues met filing threshold during this run).

## Acceptance Criteria Status

- [x] All scenarios tested
- [x] Results documented with screenshots
- [x] Bugs filed for any issues (none observed)
