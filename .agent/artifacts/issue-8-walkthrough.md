# Issue #8 Walkthrough

## Implemented
Updated `/web-sim/simulation.js` to support a fleet-oriented simulation architecture.

### 1. Multi-rover initialization
- Added `DEFAULT_ROVER_IDS = ["rover-1", "rover-2", "rover-3"]`.
- `SimulationController` now instantiates three `RoverNode` instances by default.
- Each rover is initialized with a unique `roverId` and independent state/position.

### 2. Namespaced rover topics
- Added `roverTopic(roverId, channel)` helper.
- Replaced single topic usage with namespaced topics:
  - command: `<roverId>:command`
  - telemetry: `<roverId>:telemetry`
  - ack: `<roverId>:ack`
- Payloads now include `rover_id` for telemetry and ACK messages.

### 3. Earth fleet state tracking
- `EarthNode` now maintains `fleetState` keyed by rover ID.
- Telemetry updates update per-rover state snapshots and emit `fleet:update`.
- Pending command entries retain rover targeting (`roverId`) for retries/timeouts.

### 4. Rover selection logic for commands
- Added `selectedRoverId` in `EarthNode` (defaults to `rover-1`).
- Added `setSelectedRover(roverId)` and `getSelectedRover()`.
- `sendCommand(cmdType, taskId, roverId?)` now routes commands to either explicit rover ID or selected rover.

### 5. Space link routing changes
- `SpaceLinkNode` now routes uplink commands to `<roverId>:command` based on `cmdData.rover_id`.
- Space link subscribes to each rover’s namespaced telemetry/ack topics and relays them to Earth events.

### 6. Telemetry monitor compatibility
- `TelemetryMonitor` now caches latest telemetry by rover.
- UI telemetry updates (`telemetry:display`) are emitted for the currently selected rover, preserving single-panel behavior.

## Validation Performed

### Syntax check
- `node --check web-sim/simulation.js`

### Runtime smoke check: initialization and command routing
- Verified:
  - rovers initialized: `rover-1`, `rover-2`, `rover-3`
  - fleet state initialized with all 3 rovers
  - selected rover defaults to `rover-1`
  - after selecting `rover-2`, commands target `rover-2`

### Runtime smoke check: independent rover state
- With deterministic link/fault settings (`dropRate=0`, `faultProbability=0`), sending `START_TASK` to `rover-2` resulted in:
  - `rover-1` remained `IDLE`
  - `rover-2` transitioned to `EXECUTING`

### Runtime smoke check: topic namespacing
- Verified listeners include namespaced topics:
  - `rover-1:command|telemetry|ack`
  - `rover-2:command|telemetry|ack`
  - `rover-3:command|telemetry|ack`

## Acceptance Criteria
- [x] 3 rovers initialized by default
- [x] Each rover has unique ID and independent state
- [x] Earth node tracks all rovers
- [x] Topic events properly namespaced

## Notes
- Also fixed an existing config fallback bug so `0` values are respected for `dropRate`, `jitter`, and `faultProbability` by using nullish coalescing (`??`) instead of `||`.
