#!/usr/bin/env python3
"""
GitHub Project Setup Script for the post-v1 Chandrayaan roadmap.

This script is the single source of truth for labels, milestones, and issues
used to track ongoing roadmap work after the Phase 1 v1.0.0 release.
"""

import json
from datetime import datetime, timezone

PROJECT_CONFIG = {
    "name": "LSOAS Chandrayaan Roadmap (Post-v1)",
    "description": (
        "Post-v1 roadmap for Chandrayaan teaching missions, lunar-base operations, "
        "and mission explainability"
    ),
    "columns": [
        {"name": "Backlog", "purpose": "Future work not yet prioritized"},
        {"name": "Ready", "purpose": "Ready to start"},
        {"name": "In Progress", "purpose": "Currently being worked on"},
        {"name": "In Review", "purpose": "Awaiting review and validation"},
        {"name": "Done", "purpose": "Completed and verified"},
    ],
    "phase1_release": {
        "tag": "v1.0.0",
        "release_pr": "#52",
        "released_on": "2026-02-17",
    },
}

LABELS = [
    # Phase labels
    {"name": "phase-v2-core", "color": "0E8A16", "description": "Chandrayaan v2 core architecture"},
    {"name": "phase-v2-sim", "color": "1D76DB", "description": "Web simulation and UX"},
    {"name": "phase-v2-docs", "color": "5319E7", "description": "Documentation and migration"},
    {"name": "phase-v2-validation", "color": "E99695", "description": "Validation and QA"},
    {"name": "phase-v2-phase2", "color": "C5DEF5", "description": "Phase 2 post-v1 roadmap"},

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
        "title": "Phase 2: Chandrayaan Teaching & Base Operations",
        "description": "Post-v1 expansion: guided mission curriculum, base-build operations, multi-sol planning, and learning analytics",
        "due_on": "2026-08-31T00:00:00Z",
    },
]

PHASE2_ISSUES = [
    {
        "title": "[EPIC] Phase 2: Chandrayaan teaching missions and lunar-base operations",
        "body": """## Context
Phase 1 shipped in v1.0.0 with context-aware tasks, dynamic risk, and the redesigned teaching dashboard.

## Phase 2 Goals
- Expand Chandrayaan lesson flows into guided mission packs with instructor controls
- Model base-predeploy and early base-build operations with realistic constraints
- Add multi-sol planning, what-if analysis, and after-action review tooling

## Success Criteria
- [ ] Mission presets support guided teaching progression and override synchronization
- [ ] Base operations tasks and constraints are represented in simulation and telemetry
- [ ] Operator can review mission replay with assignment/risk rationale
- [ ] Phase 2 acceptance tests and docs are in place
""",
        "labels": [
            "phase-v2-phase2",
            "type: epic",
            "priority: P0-critical",
            "category: architecture",
            "mission-phase: base-build",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
    {
        "title": "Mission preset packs v2: guided lesson flow + control synchronization",
        "body": """## Task
Add curated Chandrayaan teaching mission packs where Mission Explanation selections always synchronize Task Configuration defaults and open Mission Controls automatically.

## Acceptance Criteria
- [ ] Preset selection updates task type, difficulty, target site, and rover strategy
- [ ] Apply preset and Use next step produce deterministic command payloads
- [ ] Manual override remains possible without breaking preset progression
- [ ] UI tests verify sync behavior and state persistence
""",
        "labels": [
            "phase-v2-phase2",
            "type: feature",
            "priority: P1-high",
            "category: web-dashboard",
            "difficulty: L3",
            "mission-phase: CY3-ops",
            "mission-phase: CY4-sample-chain",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
    {
        "title": "Base-build operations pack: regolith logistics, excavation, and emplacement tasks",
        "body": """## Task
Extend task catalog and scenario templates for realistic base-predeploy/base-build workflows (site prep, regolith movement, equipment emplacement).

## Acceptance Criteria
- [ ] New base-build task templates map to existing task families and difficulty L1-L5
- [ ] Capability constraints reject infeasible assignments deterministically
- [ ] Telemetry exposes base-build context and predicted risk
- [ ] Sample scenarios documented in README and docs
""",
        "labels": [
            "phase-v2-phase2",
            "type: feature",
            "priority: P0-critical",
            "category: architecture",
            "category: ros",
            "difficulty: L4",
            "task-type: digging",
            "task-type: pushing",
            "task-type: sample-handling",
            "mission-phase: base-predeploy",
            "mission-phase: base-build",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
    {
        "title": "Multi-sol energy and thermal planner for assignment pre-check",
        "body": """## Task
Add a planning layer that forecasts battery, solar, and thermal margins across multiple lunar cycles before dispatching higher-risk tasks.

## Acceptance Criteria
- [ ] Planner produces feasibility score for next mission window
- [ ] Assignment logic can consume planner output as a weighted factor
- [ ] Night-time and low-solar windows visibly affect assignment decisions
- [ ] Tests cover planner edge cases and fallback behavior
""",
        "labels": [
            "phase-v2-phase2",
            "type: enhancement",
            "priority: P1-high",
            "category: ros",
            "category: testing",
            "difficulty: L4",
            "mission-phase: LUPEX-prospecting",
            "mission-phase: base-build",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
    {
        "title": "Mission replay and explainability timeline for teaching outcomes",
        "body": """## Task
Implement timeline replay that explains why rover assignments were chosen and how risk/context changed over time.

## Acceptance Criteria
- [ ] Replay shows command, assignment score breakdown, and telemetry checkpoints
- [ ] Instructor can filter by rover, task type, and mission phase
- [ ] Exportable summary supports post-mission teaching review
- [ ] Web and ROS telemetry semantics stay aligned
""",
        "labels": [
            "phase-v2-phase2",
            "type: feature",
            "priority: P1-high",
            "category: web-dashboard",
            "category: documentation",
            "difficulty: L3",
            "mission-phase: CY4-sample-chain",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
    {
        "title": "Phase 2 validation matrix and release criteria (v1.1.0 target)",
        "body": """## Task
Define and implement the Phase 2 validation matrix spanning catalog extensions, planning logic, UX synchronization, and replay telemetry.

## Acceptance Criteria
- [ ] Test matrix covers positive/negative assignment paths for new scenarios
- [ ] Web dashboard interaction tests cover collapsible panels and preset sync
- [ ] Regression tests preserve Phase 1 command/task behavior
- [ ] Release checklist documented for v1.1.0
""",
        "labels": [
            "phase-v2-phase2",
            "type: feature",
            "priority: P0-critical",
            "category: testing",
            "category: documentation",
            "difficulty: L3",
        ],
        "milestone": "Phase 2: Chandrayaan Teaching & Base Operations",
    },
]

ALL_ISSUES = PHASE2_ISSUES

PHASE1_COMPLETED_ISSUES = [38, 39, 40, 41, 42, 43, 44, 45, 46]
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

    commands.append("\n# Close completed Phase 1 issues")
    phase1_comment = (
        "Completed in Phase 1 and released in v1.0.0 (PR #52, tag v1.0.0). "
        "This issue is now closed as delivered."
    )
    for issue_number in PHASE1_COMPLETED_ISSUES:
        commands.append(
            f"gh issue close {issue_number} --comment \"{phase1_comment}\" || true"
        )

    commands.append("\n# Ensure legacy roadmap issues stay superseded")
    legacy_comment = (
        "Superseded by the Chandrayaan roadmap refresh and completed Phase 1 baseline. "
        "Track active roadmap work under the Phase 2 epic and child issues."
    )
    for issue_number in SUPERSCEDED_LEGACY_ISSUES:
        commands.append(
            f"gh issue close {issue_number} --comment \"{legacy_comment}\" || true"
        )

    return commands


def build_payload():
    return {
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "project_config": PROJECT_CONFIG,
        "labels": LABELS,
        "milestones": MILESTONES,
        "issues": ALL_ISSUES,
        "phase1_completed_issue_numbers": PHASE1_COMPLETED_ISSUES,
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
