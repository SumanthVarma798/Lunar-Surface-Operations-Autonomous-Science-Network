# Chandrayaan v2 Migration Notes

## Superseded Legacy Issues

Closed as superseded by new roadmap epic:

- `#2`
- `#16`
- `#17`
- `#18`
- `#19`
- `#20`

Replacement roadmap starts at:

- `#38` (epic) with child issues `#39` to `#46`.

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
