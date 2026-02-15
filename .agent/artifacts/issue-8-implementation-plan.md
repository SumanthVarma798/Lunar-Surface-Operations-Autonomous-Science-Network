# Issue #8 Implementation Plan

## Issue
**Title:** Update simulation.js for multi-rover architecture

## Goals
1. Initialize 3 rover instances by default.
2. Give each rover a unique identity and independent runtime state.
3. Track per-rover fleet state in Earth node.
4. Namespace rover event topics so each rover has isolated telemetry/command/ack channels.

## Planned Changes (`web-sim/simulation.js`)

1. Add topic helpers and rover ID constants
- Introduce default rover IDs (`rover-1`, `rover-2`, `rover-3`).
- Add utility functions to build namespaced topic names, e.g. `<roverId>:telemetry`, `<roverId>:command`, `<roverId>:ack`.

2. Refactor `RoverNode` for rover-specific channels
- Add `roverId` as a required identity for each rover instance.
- Subscribe to `<roverId>:command` only.
- Publish telemetry to `<roverId>:telemetry` and ACKs to `<roverId>:ack`.
- Include `rover_id` in telemetry and ACK payloads.

3. Update `SpaceLinkNode` relay wiring for fleet events
- Keep Earth uplink event (`earth:uplink_cmd`) as ingress.
- Route uplink commands by `rover_id` onto `<roverId>:command`.
- Relay namespaced telemetry/ack topics to Earth (`earth:telemetry`, `earth:ack`) while preserving `rover_id` in payload.

4. Expand `EarthNode` to fleet-aware command and state handling
- Add `fleetState` map keyed by rover ID.
- Update telemetry callback to refresh `fleetState[rover_id]`.
- Keep pending command retries, while preserving target `rover_id` in each pending entry.
- Add rover selection support:
  - `selectedRoverId` defaulting to `rover-1`.
  - `setSelectedRover(roverId)` validation.
  - `sendCommand(cmdType, taskId, roverId)` where explicit `roverId` overrides selected rover.

5. Update `TelemetryMonitor` and `SimulationController`
- Keep compatibility with existing UI by still emitting `telemetry:display` for selected rover updates.
- Add controller-level fleet objects:
  - `this.rovers` array with 3 `RoverNode` instances.
  - `getFleetState()` and `setSelectedRover(roverId)` helpers.
- Preserve existing public APIs where possible (`sendCommand(type, taskId)` still works and targets selected rover).

## Verification Plan
1. Static check by loading `web-sim/index.html` and ensuring no runtime errors.
2. Command-path sanity:
- `sim.sendCommand("START_TASK", "TASK-001")` targets selected rover (default `rover-1`).
- `sim.setSelectedRover("rover-2")` then command affects rover-2 only.
3. Fleet telemetry sanity:
- Three telemetry streams are active.
- `EarthNode.fleetState` contains 3 rover entries.
4. Acceptance criteria check:
- [ ] 3 rovers initialized by default
- [ ] Each rover has unique ID and independent state
- [ ] Earth node tracks all rovers
- [ ] Topic events properly namespaced
