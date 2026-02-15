#!/usr/bin/env python3
"""
GitHub Project Setup Script for LSOAS Mission Roadmap
Creates a comprehensive project board with all phases, issues, and labels
"""

import json
import os

# Project configuration
PROJECT_CONFIG = {
    "name": "LSOAS Mission Roadmap",
    "description": "Professional development roadmap for Lunar Surface Operations Autonomous Science Network - From single rover to multi-asset space mission control system",
    "columns": [
        {"name": "📋 Backlog", "purpose": "Future work not yet prioritized"},
        {"name": "📅 Ready", "purpose": "Ready to start, prioritized"},
        {"name": "🚧 In Progress", "purpose": "Currently being worked on"},
        {"name": "👀 In Review", "purpose": "Awaiting code review or testing"},
        {"name": "✅ Done", "purpose": "Completed and verified"}
    ]
}

# Labels for the repository
LABELS = [
    # Phase labels
    {"name": "phase-1", "color": "0E8A16", "description": "Phase 1: Multi-Rover Constellation"},
    {"name": "phase-2", "color": "1D76DB", "description": "Phase 2: Resource Management"},
    {"name": "phase-3", "color": "5319E7", "description": "Phase 3: Ground Station Network"},
    {"name": "phase-4", "color": "E99695", "description": "Phase 4: Mission Planning"},
    {"name": "phase-5", "color": "D93F0B", "description": "Phase 5: Advanced Coordination"},
    
    # Complexity labels
    {"name": "complexity: low", "color": "D4C5F9", "description": "Low complexity task"},
    {"name": "complexity: medium", "color": "C2E0C6", "description": "Medium complexity task"},
    {"name": "complexity: high", "color": "FBCA04", "description": "High complexity task"},
    {"name": "complexity: very-high", "color": "D73A4A", "description": "Very high complexity task"},
    
    # Category labels
    {"name": "category: ros", "color": "006B75", "description": "ROS 2 implementation work"},
    {"name": "category: web-dashboard", "color": "0075CA", "description": "Web simulation dashboard"},
    {"name": "category: documentation", "color": "0E8A16", "description": "Documentation updates"},
    {"name": "category: testing", "color": "D876E3", "description": "Testing and verification"},
    {"name": "category: architecture", "color": "5319E7", "description": "System architecture design"},
    
    # Priority labels
    {"name": "priority: P0-critical", "color": "B60205", "description": "Blocking issue, must fix ASAP"},
    {"name": "priority: P1-high", "color": "D93F0B", "description": "High priority"},
    {"name": "priority: P2-medium", "color": "FBCA04", "description": "Medium priority"},
    {"name": "priority: P3-low", "color": "0E8A16", "description": "Low priority, nice to have"},
    
    # Type labels
    {"name": "type: epic", "color": "3E4B9E", "description": "Large feature spanning multiple issues"},
    {"name": "type: feature", "color": "84B6EB", "description": "New feature"},
    {"name": "type: enhancement", "color": "A2EEEF", "description": "Enhancement to existing feature"},
    {"name": "type: bug", "color": "D73A4A", "description": "Bug fix"},
    {"name": "type: research", "color": "D4C5F9", "description": "Research and investigation"},
    
    # Duration labels
    {"name": "duration: 1-3 days", "color": "C5DEF5", "description": "Estimated 1-3 days"},
    {"name": "duration: 1 week", "color": "BFD4F2", "description": "Estimated 1 week"},
    {"name": "duration: 2-3 weeks", "color": "528BCE", "description": "Estimated 2-3 weeks"},
    {"name": "duration: 1+ month", "color": "1E6DB7", "description": "Estimated 1+ month"}
]

# Milestones
MILESTONES = [
    {
        "title": "Phase 1: Multi-Rover Constellation",
        "description": "Transform from single-rover to fleet management system with 3-5 rovers operating simultaneously",
        "due_on": "2026-04-01T00:00:00Z"
    },
    {
        "title": "Phase 2: Resource Management",
        "description": "Implement power, thermal, communication budget, and consumables tracking",
        "due_on": "2026-05-15T00:00:00Z"
    },
    {
        "title": "Phase 3: Ground Station Network",
        "description": "Multi-station coverage windows and handoff logic",
        "due_on": "2026-06-30T00:00:00Z"
    },
    {
        "title": "Phase 4: Mission Planning",
        "description": "Command sequencing and autonomous mission planning",
        "due_on": "2026-08-31T00:00:00Z"
    },
    {
        "title": "Phase 5: Advanced Coordination",
        "description": "Swarm behaviors, collision avoidance, and AI decision-making",
        "due_on": "2026-11-30T00:00:00Z"
    }
]

# Issues for Phase 1
PHASE_1_ISSUES = [
    {
        "title": "[EPIC] Phase 1: Multi-Rover Constellation",
        "body": """## Overview
Transform LSOAS from single-rover to fleet management system supporting 3-5 simultaneous rovers.

## Goals
- Multi-rover ROS architecture with unique IDs
- Fleet-level task assignment and orchestration  
- Enhanced dashboard with fleet status grid
- Rover-to-rover data relay capability

## Success Criteria
- [ ] 3-5 rovers operating simultaneously
- [ ] Auto task assignment based on battery + state
- [ ] Fleet dashboard showing all rovers
- [ ] Manual rover selection override
- [ ] All tests passing

## Timeline
4-6 weeks

## Dependencies
None (foundation phase)

## Resources
- [Phase 1 Implementation Plan](../blob/main/docs/phase1_implementation_plan.md)
""",
        "labels": ["phase-1", "type: epic", "priority: P0-critical", "complexity: medium"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    # ROS Implementation
    {
        "title": "Refactor rover_node.py for multi-rover support",
        "body": """## Task
Update `rover_node.py` to support multiple rover instances with unique IDs.

## Changes Needed
- Add `rover_id` parameter to constructor
- Update topic names: `/rover/{id}/downlink_telemetry`, `/rover/{id}/command`, `/rover/{id}/ack`
- Add telemetry fields: `rover_id`, `position` (lat/lon), `solar_exposure`, `data_buffer_size`

## Acceptance Criteria
- [ ] Rover accepts `rover_id` parameter
- [ ] Topics include rover ID in namespace
- [ ] Telemetry includes new fields
- [ ] Multiple rovers can run simultaneously without conflicts

## Files
- `lunar_ops/rover_ws/src/rover_core/rover_core/rover_node.py`
""",
        "labels": ["phase-1", "category: ros", "type: feature", "priority: P0-critical", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Add fleet management to earth_node.py",
        "body": """## Task
Implement fleet registry and task assignment logic in `earth_node.py`.

## Changes Needed
- Add fleet registry tracking all rover states
- Implement auto-assignment algorithm (select rover with highest battery + IDLE state)
- Subscribe to telemetry from all rovers dynamically
- Add `ASSIGN_TASK <rover_id> <task_id>` command with manual/auto modes

## Acceptance Criteria
- [ ] Earth node tracks all rover states
- [ ] Auto-assignment selects optimal rover
- [ ] Manual assignment works for specific rover
- [ ] Rejects commands if no suitable rover available

## Files
- `lunar_ops/rover_ws/src/rover_core/rover_core/earth_node.py`
""",
        "labels": ["phase-1", "category: ros", "type: feature", "priority: P0-critical", "complexity: medium", "duration: 1 week"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Update space_link_node.py for multi-rover relay",
        "body": """## Task
Enable space link to relay messages for multiple rovers.

## Changes Needed
- Subscribe to `/rover/*/command` using wildcard patterns (or dynamic subscriptions)
- Track relay statistics per rover
- Handle bidirectional relay for all rovers

## Acceptance Criteria
- [ ] Relays commands to all rovers
- [ ] Relays telemetry from all rovers
- [ ] Per-rover statistics tracked
- [ ] No message conflicts or drops

## Files
- `lunar_ops/rover_ws/src/rover_core/rover_core/space_link_node.py`
""",
        "labels": ["phase-1", "category: ros", "type: enhancement", "priority: P1-high", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Create fleet_manager.py node",
        "body": """## Task
Create new ROS node for centralized fleet orchestration and monitoring.

## Features
- Subscribe to all rover telemetry streams
- Aggregate fleet-level statistics (total rovers, state distribution, avg battery)
- Auto-recovery logic (configurable)
- Publish `/fleet/status` topic with fleet summary

## Acceptance Criteria
- [ ] Tracks all rover states in real-time
- [ ] Fleet status topic published at 1 Hz
- [ ] Auto-recovery can be enabled/disabled
- [ ] Clean logging of fleet events

## Files
- `lunar_ops/rover_ws/src/rover_core/rover_core/fleet_manager.py` (NEW)
- `lunar_ops/rover_ws/src/rover_core/setup.py` (add entry point)
""",
        "labels": ["phase-1", "category: ros", "type: feature", "priority: P1-high", "complexity: medium", "duration: 1 week"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Create ROS 2 launch file for fleet",
        "body": """## Task
Create launch file to start all nodes for multi-rover configuration.

## Features
- Launch space link node
- Launch 3 rover nodes with IDs (rover-1, rover-2, rover-3)
- Launch fleet manager
- Launch earth station
- Launch telemetry monitor

## Acceptance Criteria
- [ ] Single command starts entire fleet
- [ ] All nodes have proper namespacing
- [ ] Parameters passed correctly
- [ ] Can configure number of rovers

## Files
- `lunar_ops/rover_ws/src/rover_core/launch/fleet_launch.py` (NEW)
""",
        "labels": ["phase-1", "category: ros", "type: feature", "priority: P2-medium", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    # Web Dashboard Implementation
    {
        "title": "Update simulation.js for multi-rover architecture",
        "body": """## Task
Refactor JavaScript simulation to support multiple rover instances.

## Changes Needed
- Instantiate 3 `RoverNode` objects with unique IDs
- Update topic naming to match ROS changes
- Modify `EarthNode` to track fleet state
- Add rover selection logic

## Acceptance Criteria
- [ ] 3 rovers initialized by default
- [ ] Each rover has unique ID and state
- [ ] Earth node tracks all rovers
- [ ] Topic events properly namespaced

## Files
- `web-sim/simulation.js`
""",
        "labels": ["phase-1", "category: web-dashboard", "type: feature", "priority: P0-critical", "complexity: medium", "duration: 1 week"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Create fleet status grid in dashboard",
        "body": """## Task
Replace single rover status panel with grid layout showing all rovers.

## UI Design
```
┌─────────┬─────────┬─────────┐
│ ROVER-1 │ ROVER-2 │ ROVER-3 │
│  IDLE   │EXECUTING│  SAFE   │
│ 🔋 87%  │ 🔋 45%  │ 🔋 23%  │
│ ☀️ Sun  │ 🌑 Dark │ ☀️ Sun  │
└─────────┴─────────┴─────────┘
```

## Acceptance Criteria
- [ ] Grid layout responsive (works on different screen sizes)
- [ ] Each card shows: state, battery, solar exposure, position
- [ ] State-based color coding
- [ ] Real-time telemetry updates

## Files
- `web-sim/index.html`
- `web-sim/index.css`
- `web-sim/app.js`
""",
        "labels": ["phase-1", "category: web-dashboard", "type: feature", "priority: P0-critical", "complexity: medium", "duration: 1 week"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Add rover selection dropdown to command panel",
        "body": """## Task
Add UI control to select which rover receives commands.

## Features
- Dropdown: Auto-Select, Rover-1, Rover-2, Rover-3
- Auto-select mode uses assignment algorithm
- Manual mode sends to specific rover
- Display selected rover in command log

## Acceptance Criteria
- [ ] Dropdown populated with rover IDs
- [ ] Auto-select works correctly
- [ ] Manual selection overrides auto logic
- [ ] Command log shows target rover

## Files
- `web-sim/index.html`
- `web-sim/app.js`
""",
        "labels": ["phase-1", "category: web-dashboard", "type: feature", "priority: P1-high", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Add fleet summary banner",
        "body": """## Task
Display aggregate fleet statistics at top of dashboard.

## Content
- Total rovers count
- State distribution (X IDLE, Y EXECUTING, Z SAFE_MODE)
- Average battery level
- Recent commands count

## Acceptance Criteria
- [ ] Banner always visible
- [ ] Updates in real-time
- [ ] Styling matches design system
- [ ] Dark/light theme support

## Files
- `web-sim/index.html`
- `web-sim/index.css`
- `web-sim/app.js`
""",
        "labels": ["phase-1", "category: web-dashboard", "type: feature", "priority: P2-medium", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    # Testing
    {
        "title": "Write unit tests for task assignment algorithm",
        "body": """## Task
Test suite for fleet task assignment logic.

## Test Cases
- Selects rover with highest battery when multiple IDLE
- Rejects if all rovers are EXECUTING or SAFE_MODE
- Considers solar exposure in scoring
- Manual assignment overrides auto logic
- Handles edge cases (single rover, all low battery)

## Acceptance Criteria
- [ ] All test cases implemented
- [ ] Tests pass consistently
- [ ] Code coverage > 80% for assignment logic

## Files
- `lunar_ops/rover_ws/src/rover_core/test/test_task_assignment.py` (NEW)
""",
        "labels": ["phase-1", "category: testing", "type: feature", "priority: P1-high", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Integration test: Multi-rover command flow",
        "body": """## Task
End-to-end test for multi-rover operations.

## Test Scenario
1. Launch 3 rovers + space link + earth
2. Send START_TASK with auto-assignment
3. Verify exactly 1 rover executes
4. Verify ACK received with correct rover_id
5. Send second task while first is running
6. Verify assigned to different rover

## Acceptance Criteria
- [ ] Test launches full fleet
- [ ] Verifies correct rover selection
- [ ] Checks ACK routing
- [ ] Teardown cleans up properly

## Files
- `lunar_ops/rover_ws/src/rover_core/test/test_fleet_integration.py` (NEW)
""",
        "labels": ["phase-1", "category: testing", "type: feature", "priority: P1-high", "complexity: medium", "duration: 1 week"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Manual testing scenarios for web dashboard",
        "body": """## Task
Document and execute manual test scenarios for fleet dashboard.

## Scenarios
1. Basic fleet operations (auto task assignment)
2. Manual rover selection
3. Safe mode handling across fleet
4. Battery-based selection verification
5. Telemetry load test (5 rovers at 0.5 Hz)

## Deliverable
Checklist in phase1_implementation_plan.md with results

## Acceptance Criteria
- [ ] All scenarios tested
- [ ] Results documented
- [ ] Screenshots captured
- [ ] Bugs filed for any issues

## Files
- `docs/phase1_testing_results.md` (NEW)
""",
        "labels": ["phase-1", "category: testing", "type: feature", "priority: P2-medium", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    # Documentation
    {
        "title": "Update README with multi-rover instructions",
        "body": """## Task
Update main README to document multi-rover capabilities.

## Changes
- Update architecture diagram
- Add fleet management description
- Update quick start for multi-rover launch
- Add fleet command examples
- Update screenshots

## Acceptance Criteria
- [ ] README accurately describes fleet system
- [ ] Launch commands tested
- [ ] Screenshots current
- [ ] Examples work as written

## Files
- `README.md`
- `docs/screenshots/` (new fleet screenshots)
""",
        "labels": ["phase-1", "category: documentation", "type: enhancement", "priority: P2-medium", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    },
    
    {
        "title": "Update Makefile for fleet operations",
        "body": """## Task
Add Makefile targets for multi-rover launch and testing.

## New Targets
- `make fleet` - Launch 3-rover fleet
- `make fleet-5` - Launch 5-rover fleet
- `make test-fleet` - Run fleet integration tests

## Acceptance Criteria
- [ ] Targets work as documented
- [ ] Help text updated
- [ ] Dependencies correct

## Files
- `Makefile`
""",
        "labels": ["phase-1", "category: documentation", "type: enhancement", "priority: P3-low", "complexity: low", "duration: 1-3 days"],
        "milestone": "Phase 1: Multi-Rover Constellation"
    }
]

# Summary issues for future phases (epics only for now)
FUTURE_PHASE_EPICS = [
    {
        "title": "[EPIC] Phase 2: Resource Management Systems",
        "body": """## Overview
Implement realistic resource constraints: power, thermal, communication bandwidth, consumables.

## Components
- Power system (solar panels, battery cycling, heaters)
- Thermal management (component temps, RHUs, operational ranges)
- Communication budget (data rate limits, prioritization, compression)
- Consumables tracking (fuel, drill cycles, sample containers)

## Timeline
3-4 weeks

## Dependencies
- Phase 1 completion
""",
        "labels": ["phase-2", "type: epic", "priority: P1-high", "complexity: medium", "duration: 2-3 weeks"],
        "milestone": "Phase 2: Resource Management"
    },
    
    {
        "title": "[EPIC] Phase 3: Ground Station Network",
        "body": """## Overview
Multi-station coverage modeling with visibility windows and handoffs.

## Components
- 3-5 ground stations at global locations
- Coverage window calculation (Earth rotation)
- Handoff logic between stations
- Antenna scheduling and conflicts
- 3D visualization of coverage

## Timeline
2-3 weeks

## Dependencies
- Phase 1 completion
""",
        "labels": ["phase-3", "type: epic", "priority: P2-medium", "complexity: medium", "duration: 2-3 weeks"],
        "milestone": "Phase 3: Ground Station Network"
    },
    
    {
        "title": "[EPIC] Phase 4: Autonomous Mission Planning",
        "body": """## Overview
Command sequencing, activity timeline planning, and autonomous replanning.

## Components
- Command sequence upload (24-hour timelines)
- Conditional logic ("if battery > 50%, then...")
- Gantt chart timeline editor
- Science campaign manager
- Autonomous re-planning on failures

## Timeline
4-5 weeks

## Dependencies
- Phase 2 completion (need resource constraints)
""",
        "labels": ["phase-4", "type: epic", "priority: P2-medium", "complexity: high", "duration: 1+ month"],
        "milestone": "Phase 4: Mission Planning"
    },
    
    {
        "title": "[EPIC] Phase 5: Advanced Fleet Coordination & AI",
        "body": """## Overview
Cutting-edge multi-agent coordination and AI decision-making.

## Components
- Swarm behaviors (formation flying, leader-follower)
- Collision avoidance (predict close approaches)
- Optical inter-rover links (mesh network)
- Reinforcement learning for path planning
- Anomaly detection and predictive maintenance
- Autonomous science prioritization

## Timeline
6-8 weeks

## Dependencies
- Phase 3 completion
- Phase 4 recommended
""",
        "labels": ["phase-5", "type: epic", "priority: P3-low", "complexity: very-high", "duration: 1+ month"],
        "milestone": "Phase 5: Advanced Coordination"
    }
]

# Combine all issues
ALL_ISSUES = PHASE_1_ISSUES + FUTURE_PHASE_EPICS

def generate_github_commands():
    """Generate gh CLI commands to create the entire project structure"""
    commands = []
    
    # Create labels
    commands.append("# Create labels")
    for label in LABELS:
        cmd = f"gh label create \"{label['name']}\" --color {label['color']} --description \"{label['description']}\" || true"
        commands.append(cmd)
    
    commands.append("\n# Create milestones")
    for i, milestone in enumerate(MILESTONES, 1):
        cmd = f"gh api repos/{{owner}}/{{repo}}/milestones -f title=\"{milestone['title']}\" -f description=\"{milestone['description']}\" -f due_on=\"{milestone['due_on']}\" || true"
        commands.append(cmd)
    
    commands.append("\n# Create issues")
    for issue in ALL_ISSUES:
        labels_str = ",".join(issue['labels'])
        # Escape quotes in body
        body_escaped = issue['body'].replace('"', '\\"').replace('\n', '\\n')
        
        # Get milestone number (simplified - in real usage would query API)
        milestone_map = {m['title']: i+1 for i, m in enumerate(MILESTONES)}
        milestone_num = milestone_map.get(issue['milestone'], 1)
        
        cmd = f"gh issue create --title \"{issue['title']}\" --body \"{body_escaped[:500]}...\" --label \"{labels_str}\" --milestone {milestone_num}"
        commands.append(cmd)
    
    return commands

if __name__ == "__main__":
    # Export to JSON for reference
    output = {
        "project_config": PROJECT_CONFIG,
        "labels": LABELS,
        "milestones": MILESTONES,
        "issues": ALL_ISSUES
    }
    
    with open("github_project_structure.json", "w") as f:
        json.dump(output, f, indent=2)
    
    print("✅ Generated github_project_structure.json")
    print(f"📊 {len(LABELS)} labels, {len(MILESTONES)} milestones, {len(ALL_ISSUES)} issues")
    
    # Generate shell script
    commands = generate_github_commands()
    with open("setup_github_project.sh", "w") as f:
        f.write("#!/bin/bash\n")
        f.write("# GitHub Project Setup for LSOAS Mission Roadmap\n")
        f.write("# Run from repository root\n\n")
        f.write("set -e\n\n")
        f.write("\n".join(commands))
    
    print("✅ Generated setup_github_project.sh")
    print("\nNext steps:")
    print("1. Review github_project_structure.json")
    print("2. Run: chmod +x setup_github_project.sh")
    print("3. Run: ./setup_github_project.sh")
