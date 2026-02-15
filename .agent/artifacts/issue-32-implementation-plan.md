# Issue #32 Implementation Plan

## Issue
**Title:** Fix fleet telemetry grid visibility/update regression

## Root Cause
The fleet telemetry grid is currently hidden whenever the 3D panel is open (`.panel-telemetry.compact-log` mode), which makes the feature appear broken. This is caused by a CSS rule that sets `.fleet-grid-panel` to `display: none` in compact mode.

## Goals
1. Keep the fleet telemetry grid visible in compact mode.
2. Preserve real-time card updates for all rovers.
3. Default to top-3 attention rovers while allowing expansion to all rovers.
4. Allow pinning/locking rovers of interest so they persist in view.
5. In 3D expanded mode, switch rover status interactions to hover mode.
6. Ensure responsive behavior remains clean on desktop/mobile.
7. Keep dark/light theme rendering correct.

## Planned Changes

### 1) `web-sim/index.css`
- Remove/harden the compact-mode rule that hides `.fleet-grid-panel`.
- Add compact-mode sizing constraints for `.fleet-grid-panel` / `.fleet-grid` so it remains visible without crowding the telemetry feed.
- Verify responsive breakpoints still produce readable cards.

### 2) `web-sim/app.js`
- Add attention scoring and priority-list logic for top-3 rovers needing attention.
- Add expand/collapse control for priority vs full fleet list.
- Add rover pinning persistence (`localStorage`) and lock pinned rovers into priority view.
- Add compact-mode hover-card behavior tied to 3D panel state.
- Add a defensive sync path to refresh fleet cards from `sim.getFleetState()` periodically in case event bursts are dropped, ensuring table remains live.
- Keep existing event-driven updates as the primary path.
- Preserve existing command routing and telemetry-feed behavior.

### 3) `web-sim/index.html`
- Add fleet grid toolbar controls (`show all / show priority`) and summary text.
- Add hover-card container used in compact/3D mode.

## Verification Plan
1. `node --check web-sim/app.js`
2. `node --check web-sim/simulation.js`
3. Local HTTP smoke check on `web-sim/index.html` to confirm fleet-grid elements load.
4. Runtime smoke check:
- 3 rovers present in fleet state.
- START_TASK transitions target rover state while others remain independent.
- Fleet grid update path remains active in compact-mode conditions.

## Acceptance Criteria Checklist
- [ ] Fleet telemetry grid is visible in compact mode
- [ ] Fleet cards continue real-time updates (state, battery, solar exposure, task progress)
- [ ] Priority mode shows top-3 attention rovers with expand-to-all option
- [ ] Pinned rovers remain locked in visible list
- [ ] Compact 3D mode supports rover-hover status card
- [ ] Layout remains responsive across desktop/mobile
- [ ] Dark/light theme remains correct
