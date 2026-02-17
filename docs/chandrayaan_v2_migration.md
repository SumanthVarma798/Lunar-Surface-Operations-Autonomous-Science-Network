# Chandrayaan v2 Migration Notes

## Superseded Legacy Issues

Closed as superseded by new roadmap epic:

- `#2`
- `#16`
- `#17`
- `#18`
- `#19`
- `#20`

Phase 1 implementation roadmap started at:

- `#38` (epic) with child issues `#39` to `#46`.

## Release Baseline

Phase 1 is released and frozen as:

- tag: `v1.0.0`
- release PR: `#52`

All Phase 1 roadmap issues `#38` to `#46` are now closed as delivered.

## Active Roadmap Track

Phase 2 roadmap now starts at:

- `#53` (epic) with child issues `#54` to `#58`.

## Behavior Changes

1. Generic fixed 10-step tasks replaced with catalog-driven durations.
2. Flat fault probability replaced with context-aware risk model.
3. Auto assignment now uses capability and risk thresholds.
4. `START_TASK` payload extended with task metadata.
5. Telemetry expanded for mission-context observability.

## Backward Compatibility

Legacy `START_TASK` payload with only `task_id` still works and defaults to:

- `task_type=movement`
- `difficulty_level=L2`
