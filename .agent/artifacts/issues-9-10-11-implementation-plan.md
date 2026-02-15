# Issues #9, #10, #11 Implementation Plan

## Scope
Implement three dashboard issues in one cohesive update:
- #9: Fleet status grid
- #10: Rover selection dropdown + auto/manual targeting
- #11: Fleet summary banner

## Files
- `web-sim/index.html`
- `web-sim/index.css`
- `web-sim/app.js`

## Planned Changes

### 1) `index.html`
- Add a top fleet summary banner directly under the top bar.
  - Metrics slots: total rovers, state distribution, average battery, commands/ACKs.
- Replace the single-rover telemetry status card with a fleet grid container.
  - Card content per rover: state, battery, solar exposure indicator, task progress.
- Add rover target dropdown in command panel.
  - Options: `Auto-Select`, `Rover-1`, `Rover-2`, `Rover-3`.

### 2) `index.css`
- Add banner styling with theme-variable colors so both dark/light modes render correctly.
- Add responsive fleet grid styles:
  - Desktop: multi-column cards.
  - Tablet/mobile: wraps down to 2/1 columns.
- Add fleet-card state styles with explicit acceptance mapping:
  - IDLE = green
  - EXECUTING = blue
  - SAFE_MODE = red
- Style rover dropdown and command log rover target tag.

### 3) `app.js`
- Maintain a local fleet cache keyed by `rover_id` and render cards on each update.
- Subscribe to simulation events to keep UI real-time:
  - `earth:telemetry` and/or `fleet:update` for rover card/banner updates.
  - `cmd:sent`, `earth:ack` for command/ACK counters.
- Implement rover target logic:
  - Manual mode: send commands to selected rover.
  - Auto-select mode: choose best rover via scoring algorithm (prefer IDLE + battery + solar exposure).
  - Manual selection overrides auto immediately.
- Update command log entries to display target rover.

## Auto-Select Algorithm (Issue #10)
- Inputs: current fleet snapshots (`state`, `battery`, `solar_exposure` inferred when absent).
- Selection policy:
  - `START_TASK`: choose highest-score IDLE rover.
  - `ABORT`: prefer EXECUTING rover with highest progress; else fallback to best available.
  - `GO_SAFE`: prefer EXECUTING rover; fallback best available.
  - `RESET`: prefer SAFE_MODE rover with lowest battery; fallback best available.
- Score baseline: `battery_weight + solar_bonus`.

## Verification Plan
1. Syntax checks:
- `node --check web-sim/simulation.js`
- `node --check web-sim/app.js`

2. Runtime smoke checks (Node-driven):
- Fleet data stream contains 3 rovers.
- Fleet grid render model updates for all rovers.
- Auto-select chooses eligible rover for `START_TASK`.
- Manual dropdown selection overrides auto and command targets chosen rover.
- Command/ACK counters advance with events.

3. Acceptance Criteria Checklist
### Issue #9
- [ ] Grid layout responsive
- [ ] Each card: state, battery, solar exposure, task progress
- [ ] State-based color coding (green=IDLE, blue=EXECUTING, red=SAFE_MODE)
- [ ] Real-time telemetry updates

### Issue #10
- [ ] Dropdown populated with rover IDs
- [ ] Auto-select uses best-rover algorithm
- [ ] Manual selection overrides auto
- [ ] Command log shows target rover

### Issue #11
- [ ] Banner always visible
- [ ] Real-time updates
- [ ] Dark/light theme support
