#!/usr/bin/env python3
"""
GitHub Project Setup Script for the Chandrayaan Swarm v2 Roadmap.

This script is the single source of truth for labels, milestones, and issues
used to run the Chandrayaan-oriented roadmap refresh.
"""

import json
from datetime import datetime, timezone

PROJECT_CONFIG = {
    "name": "LSOAS Chandrayaan Swarm v2 Roadmap",
    "description": (
        "Chandrayaan-themed autonomous lunar swarm roadmap: realistic mission "
        "operations plus future base-build extensions"
    ),
    "columns": [
        {"name": "Backlog", "purpose": "Future work not yet prioritized"},
        {"name": "Ready", "purpose": "Ready to start"},
        {"name": "In Progress", "purpose": "Currently being worked on"},
        {"name": "In Review", "purpose": "Awaiting review and validation"},
        {"name": "Done", "purpose": "Completed and verified"},
    ],
}

LABELS = [
    # Phase labels
    {"name": "phase-v2-core", "color": "0E8A16", "description": "Chandrayaan v2 core architecture"},
    {"name": "phase-v2-sim", "color": "1D76DB", "description": "Web simulation and UX"},
    {"name": "phase-v2-docs", "color": "5319E7", "description": "Documentation and migration"},
    {"name": "phase-v2-validation", "color": "E99695", "description": "Validation and QA"},

    # Existing broad labels retained
    {"name": "category: ros", "color": "006B75", "description": "ROS 2 implementation work"},
    {"name": "category: web-dashboard", "color": "0075CA", "description": "Web simulation dashboard"},
    {"name": "category: documentation", "color": "0E8A16", "description": "Documentation updates"},
    {"name": "category: testing", "color": "D876E3", "description": "Testing and verification"},
    {"name": "category: architecture", "color": "5319E7", "description": "System architecture design"},

    # Task taxonomy labels
    {"name": "task-type: movement", "color": "0052CC", "description": "Traverse and navigation operations"},
    {"name": "task-type: science", "color": "1D76DB", "description": "In-situ science operations"},
    {"name": "task-type: digging", "color": "8B572A", "description": "Digging and drilling operations"},
    {"name": "task-type: pushing", "color": "D93F0B", "description": "Regolith push and transport operations"},
    {"name": "task-type: photo", "color": "5319E7", "description": "Imaging and survey operations"},
    {"name": "task-type: sample-handling", "color": "FBCA04", "description": "Sample transfer and handling"},

    # Difficulty labels
    {"name": "difficulty: L1", "color": "C2E0C6", "description": "Difficulty level 1"},
    {"name": "difficulty: L2", "color": "BFDADC", "description": "Difficulty level 2"},
    {"name": "difficulty: L3", "color": "FBCA04", "description": "Difficulty level 3"},
    {"name": "difficulty: L4", "color": "F9A602", "description": "Difficulty level 4"},
    {"name": "difficulty: L5", "color": "D73A4A", "description": "Difficulty level 5"},

    # Mission phase labels
    {"name": "mission-phase: CY3-ops", "color": "0E8A16", "description": "Chandrayaan-3 style surface operations"},
    {"name": "mission-phase: CY4-sample-chain", "color": "1D76DB", "description": "Chandrayaan-4 sample-return chain"},
    {"name": "mission-phase: LUPEX-prospecting", "color": "5319E7", "description": "LUPEX-style polar prospecting"},
    {"name": "mission-phase: base-predeploy", "color": "E99695", "description": "Future base infrastructure pre-deployment"},
    {"name": "mission-phase: base-build", "color": "B60205", "description": "Future base-build operations"},

    # Capability labels
    {"name": "capability-class: mobility", "color": "0052CC", "description": "Mobility and hazard navigation"},
    {"name": "capability-class: science", "color": "1D76DB", "description": "Science payload capability"},
    {"name": "capability-class: excavation", "color": "8B572A", "description": "Excavation and drill capability"},
    {"name": "capability-class: manipulation", "color": "D93F0B", "description": "Manipulation and push/transport capability"},
    {"name": "capability-class: imaging", "color": "5319E7", "description": "Imaging and survey capability"},
    {"name": "capability-class: sample-logistics", "color": "FBCA04", "description": "Sample handling and transfer capability"},

    # Priority and type labels
    {"name": "priority: P0-critical", "color": "B60205", "description": "Blocking issue, must fix ASAP"},
    {"name": "priority: P1-high", "color": "D93F0B", "description": "High priority"},
    {"name": "priority: P2-medium", "color": "FBCA04", "description": "Medium priority"},
    {"name": "priority: P3-low", "color": "0E8A16", "description": "Low priority"},
    {"name": "type: epic", "color": "3E4B9E", "description": "Large feature spanning multiple issues"},
    {"name": "type: feature", "color": "84B6EB", "description": "New feature"},
    {"name": "type: enhancement", "color": "A2EEEF", "description": "Enhancement to existing feature"},
    {"name": "type: research", "color": "D4C5F9", "description": "Research and validation"},
]

MILESTONES = [
    {
        "title": "Chandrayaan v2 - Core Task Model",
        "description": "Task catalog, difficulty model, and assignment scoring",
        "due_on": "2026-03-25T00:00:00Z",
    },
    {
        "title": "Chandrayaan v2 - Simulation and Telemetry",
        "description": "Web simulation parity with ROS task semantics",
        "due_on": "2026-04-20T00:00:00Z",
    },
    {
        "title": "Chandrayaan v2 - Documentation and Validation",
        "description": "README, architecture docs, and test evidence",
        "due_on": "2026-05-08T00:00:00Z",
    },
]

CHANDRAYAAN_V2_ISSUES = [
    {
        "title": "[EPIC] Chandrayaan v2: Context-aware swarm task orchestration",
        "body": """## Overview
Redesign task planning and rover assignment around Chandrayaan and LUPEX-inspired operations with a future Indian lunar base scenario.

## Goals
- Structured task taxonomy with six mission families
- Difficulty levels L1-L5 with context-aware fault modeling
- Capability-aware rover assignment with explainable scoring
- Unified ROS + web-sim behavior using a shared task catalog

## Success Criteria
- [ ] Task catalog file is consumed by ROS and web-sim
- [ ] Dynamic fault model replaces fixed per-step fault chance
- [ ] Assignment returns score breakdown and reject reason when infeasible
- [ ] New telemetry fields expose mission context and risk
- [ ] Legacy open roadmap epics are superseded and linked
""",
        "labels": [
            "phase-v2-core",
            "type: epic",
            "priority: P0-critical",
            "category: architecture",
            "mission-phase: base-predeploy",
        ],
        "milestone": "Chandrayaan v2 - Core Task Model",
    },
    {
        "title": "Create shared Chandrayaan TaskCatalog schema and loader",
        "body": """## Task
Add a machine-readable TaskCatalog file and loader utilities for both ROS and web-sim.

## Required fields
- task_type
- difficulty_level
- base_fault_rate
- duration_profile
- required_capabilities

## Acceptance Criteria
- [ ] Catalog validates at startup
- [ ] L1..L5 maps to 1/3/6/10/18 percent base fault rates
- [ ] Catalog file is versioned and documented
""",
        "labels": [
            "phase-v2-core",
            "type: feature",
            "priority: P0-critical",
            "category: architecture",
            "difficulty: L3",
            "task-type: movement",
            "task-type: science",
            "task-type: digging",
            "task-type: pushing",
            "task-type: photo",
            "task-type: sample-handling",
        ],
        "milestone": "Chandrayaan v2 - Core Task Model",
    },
    {
        "title": "Implement dynamic fault-rate engine with lunar context modifiers",
        "body": """## Task
Replace flat fault chance with dynamic risk driven by mission context.

## Context inputs
- battery SOC
- lunar day/night state
- solar intensity
- terrain difficulty
- comm quality and latency
- thermal stress

## Acceptance Criteria
- [ ] Output fault probability clamped to 0-60 percent
- [ ] Same task yields different risk in day vs night and low vs high battery
- [ ] Unit tests cover modifier behavior
""",
        "labels": [
            "phase-v2-core",
            "type: feature",
            "priority: P1-high",
            "category: ros",
            "category: testing",
            "difficulty: L4",
            "mission-phase: LUPEX-prospecting",
        ],
        "milestone": "Chandrayaan v2 - Core Task Model",
    },
    {
        "title": "Add capability-class rover profiles and explainable assignment output",
        "body": """## Task
Implement rover capability classes and assignment scoring with explanation.

## Scoring factors
- capability match
- battery margin
- solar margin
- thermal margin
- comm margin
- distance and accessibility
- predicted mission risk

## Acceptance Criteria
- [ ] Assignment returns selected_rover, score_breakdown, reject_reason
- [ ] Assignment rejects infeasible tasks deterministically
- [ ] Auto mode chooses different rover under changed mission context
""",
        "labels": [
            "phase-v2-core",
            "type: feature",
            "priority: P0-critical",
            "category: ros",
            "difficulty: L4",
            "capability-class: mobility",
            "capability-class: science",
            "capability-class: excavation",
            "capability-class: manipulation",
            "capability-class: imaging",
            "capability-class: sample-logistics",
        ],
        "milestone": "Chandrayaan v2 - Core Task Model",
    },
    {
        "title": "Update rover execution engine for variable duration and risk by task+difficulty",
        "body": """## Task
Remove fixed 10-step task execution and support catalog-driven duration profiles by task and difficulty.

## Acceptance Criteria
- [ ] L1 tasks complete faster than L5 for same family
- [ ] Battery drain and risk reflect task profile
- [ ] Telemetry includes active_task_type and active_task_difficulty
""",
        "labels": [
            "phase-v2-core",
            "type: enhancement",
            "priority: P1-high",
            "category: ros",
            "difficulty: L3",
            "mission-phase: CY3-ops",
        ],
        "milestone": "Chandrayaan v2 - Core Task Model",
    },
    {
        "title": "Extend web-sim command panel with task type and difficulty controls",
        "body": """## Task
Expose task_type and difficulty_level controls in the dashboard and route them in command payloads.

## Acceptance Criteria
- [ ] Operator can set movement/science/digging/pushing/photo/sample-handling
- [ ] Operator can set L1-L5
- [ ] Auto assignment decisions are logged with score breakdown
""",
        "labels": [
            "phase-v2-sim",
            "type: feature",
            "priority: P1-high",
            "category: web-dashboard",
            "difficulty: L2",
        ],
        "milestone": "Chandrayaan v2 - Simulation and Telemetry",
    },
    {
        "title": "Publish expanded telemetry schema for mission-context observability",
        "body": """## Task
Extend rover and fleet telemetry with context and risk observability fields.

## Required fields
- active_task_type
- active_task_difficulty
- predicted_fault_probability
- assignment_score_breakdown
- lunar_time_state
- solar_intensity

## Acceptance Criteria
- [ ] ROS and web-sim publish and consume these fields
- [ ] Fleet display shows context-aware risk insights
""",
        "labels": [
            "phase-v2-sim",
            "type: enhancement",
            "priority: P1-high",
            "category: web-dashboard",
            "category: ros",
            "difficulty: L3",
        ],
        "milestone": "Chandrayaan v2 - Simulation and Telemetry",
    },
    {
        "title": "Validation suite for catalog parsing, risk engine, and assignment edge cases",
        "body": """## Task
Expand tests for task catalog validation, difficulty behavior, context modifiers, and assignment rejection paths.

## Acceptance Criteria
- [ ] Catalog parser tests
- [ ] L1 vs L5 duration/risk behavior tests
- [ ] Low battery + lunar night risk increase tests
- [ ] No feasible rover deterministic rejection tests
- [ ] Legacy START_TASK fallback behavior tests
""",
        "labels": [
            "phase-v2-validation",
            "type: feature",
            "priority: P0-critical",
            "category: testing",
            "difficulty: L3",
        ],
        "milestone": "Chandrayaan v2 - Documentation and Validation",
    },
    {
        "title": "Rewrite README and architecture docs for Chandrayaan swarm-base narrative",
        "body": """## Task
Update public docs to Chandrayaan future mission framing while separating real mission basis from future extrapolation.

## Acceptance Criteria
- [ ] README includes Reality basis section (CY3/CY4/LUPEX)
- [ ] Development workflow documents main->develop->feature->PR
- [ ] Architecture docs describe task taxonomy and risk model
""",
        "labels": [
            "phase-v2-docs",
            "type: enhancement",
            "priority: P1-high",
            "category: documentation",
            "difficulty: L2",
            "mission-phase: CY4-sample-chain",
            "mission-phase: base-build",
        ],
        "milestone": "Chandrayaan v2 - Documentation and Validation",
    },
]


ALL_ISSUES = CHANDRAYAAN_V2_ISSUES


SUPERSCEDED_LEGACY_ISSUES = [2, 16, 17, 18, 19, 20]


def generate_github_commands():
    """Generate CLI commands to apply roadmap metadata and issues."""
    commands = []
    commands.append("# Labels")
    for label in LABELS:
        commands.append(
            f"gh label create \"{label['name']}\" --color {label['color']} "
            f"--description \"{label['description']}\" || true"
        )

    commands.append("\n# Milestones")
    for milestone in MILESTONES:
        commands.append(
            "gh api repos/{owner}/{repo}/milestones "
            f"-f title=\"{milestone['title']}\" "
            f"-f description=\"{milestone['description']}\" "
            f"-f due_on=\"{milestone['due_on']}\" || true"
        )

    milestone_map = {m["title"]: idx + 1 for idx, m in enumerate(MILESTONES)}

    commands.append("\n# Issues")
    for issue in ALL_ISSUES:
        labels_str = ",".join(issue["labels"])
        body_escaped = issue["body"].replace('"', '\\"').replace("\n", "\\n")
        milestone_num = milestone_map.get(issue["milestone"], 1)
        commands.append(
            f"gh issue create --title \"{issue['title']}\" "
            f"--body \"{body_escaped}\" --label \"{labels_str}\" "
            f"--milestone {milestone_num}"
        )

    commands.append("\n# Supersede legacy roadmap issues")
    for issue_number in SUPERSCEDED_LEGACY_ISSUES:
        comment = (
            "Superseded by the Chandrayaan v2 roadmap refresh. "
            "See the new Chandrayaan v2 epic and linked child issues."
        )
        commands.append(
            f"gh issue close {issue_number} --comment \"{comment}\" || true"
        )

    return commands


def build_payload():
    return {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "project_config": PROJECT_CONFIG,
        "labels": LABELS,
        "milestones": MILESTONES,
        "issues": ALL_ISSUES,
        "superseded_legacy_issue_numbers": SUPERSCEDED_LEGACY_ISSUES,
        "commands": generate_github_commands(),
    }


def main():
    payload = build_payload()
    with open("github_project_structure.json", "w", encoding="utf-8") as fp:
        json.dump(payload, fp, indent=2)

    print("Generated github_project_structure.json")
    print(f"labels={len(LABELS)} milestones={len(MILESTONES)} issues={len(ALL_ISSUES)}")


if __name__ == "__main__":
    main()
