---
description: Implement a GitHub issue from the LSOAS roadmap using the main + develop model
---

# Implement Issue

This workflow fetches a GitHub issue and implements it end-to-end on a `codex/*` branch.

## Steps

// turbo-all

1. **Fetch the issue** from GitHub:

   ```bash
   export GH_CONFIG_DIR=/Users/varma/.gh_config
   gh issue view <ISSUE_NUMBER> \
     -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
     --json title,body,labels,milestone,state
   ```

2. **Read the issue body** and extract:
   - Goals and required behavior
   - Files to modify or create
   - Acceptance criteria checkboxes

3. **Sync branches and create a feature branch from develop**:

   ```bash
   git fetch origin
   git checkout main
   git pull --ff-only origin main
   git checkout develop || git checkout -b develop origin/main
   git pull --ff-only origin develop
   git checkout -b codex/issue-<ISSUE_NUMBER>-<short-kebab-title>
   ```

4. **Plan implementation** as a checklist mapped to acceptance criteria.

5. **Implement changes** file by file.

6. **Verify implementation**:
   - Run relevant tests/lint checks
   - If UI is impacted, verify behavior in `http://localhost:8080`
   - Confirm each acceptance criterion is satisfied

7. **Commit and push**:

   ```bash
   git add -A
   git commit -m "feat: <issue-title> (#<ISSUE_NUMBER>)"
   git push -u origin codex/issue-<ISSUE_NUMBER>-<short-kebab-title>
   ```

8. **Create PR to develop**:

   ```bash
   export GH_CONFIG_DIR=/Users/varma/.gh_config
   gh pr create \
     -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
     --base develop \
     --head codex/issue-<ISSUE_NUMBER>-<short-kebab-title> \
     --title "feat: <issue-title> (#<ISSUE_NUMBER>)" \
     --body "Closes #<ISSUE_NUMBER>"
   ```

9. **Move issue to In Review** on project board:

   ```bash
   export GH_CONFIG_DIR=/Users/varma/.gh_config
   gh project item-edit \
     --project-id <PROJECT_ID> \
     --id <ITEM_ID> \
     --field-id <STATUS_FIELD_ID> \
     --single-select-option-id <IN_REVIEW_OPTION_ID>
   ```

10. **Create walkthrough artifact** with what changed, test evidence, and screenshots if UI changed.

11. **Release path**:
   - After feature PRs are merged into `develop`, promote via release PR `develop -> main`.
   - Follow `.agent/workflows/release.md` for tagging and GitHub Release publishing.
