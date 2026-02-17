# Chandrayaan v2 Testing Guide

## Automated Tests

Run:

```bash
pytest -q lunar_ops/rover_ws/src/rover_core/test
```

## Required Scenarios

1. TaskCatalog parsing and schema checks.
2. Difficulty profile checks (L1 vs L5 duration and risk).
3. Context modifier checks (low battery + night vs high battery + day).
4. Capability-aware assignment checks (`science` vs `digging` candidate selection).
5. Deterministic rejection when no rover is feasible.
6. Legacy `START_TASK` fallback behavior.

## Manual Web-Sim Smoke Checks

1. Start web-sim.
2. Set `Task Type=Science`, `Difficulty=L3`, run auto dispatch.
3. Confirm command log shows type/difficulty and selected rover.
4. Confirm telemetry feed shows risk percentage and task metadata.
5. Raise `Risk Bias` and verify increased fault incidence.
6. Try an extreme task (`sample-handling`, `L5`) with poor rover state and confirm rejection.
