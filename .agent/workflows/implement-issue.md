---
description: Implement a GitHub issue from the LSOAS roadmap using dev, staging, and production gates
---

# Implement Issue

This workflow fetches a GitHub issue and implements it end-to-end on a `codex/*` branch.

Prerequisite: run `gh auth login` once on your machine before using `gh` commands.

## Steps

// turbo-all

1. **Fetch the issue** from GitHub:

   ```bash
   gh auth status || gh auth login
   gh issue view <ISSUE_NUMBER> \
     -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
     --json title,body,labels,milestone,state
   ```

2. **Read the issue body** and extract:
   - Goals and required behavior
   - Files to modify or create
   - Acceptance criteria checkboxes

3. **Sync and branch from dev**:

   ```bash
   git fetch origin
   git checkout main
   git pull --ff-only origin main
   git checkout staging || git checkout -b staging origin/main
   git push -u origin staging || true
   git pull --ff-only origin staging
   git checkout dev || git checkout -b dev origin/staging || git checkout -b dev staging
   git push -u origin dev || true
   git pull --ff-only origin dev
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

8. **Create PR to dev**:

   ```bash
   gh pr create \
     -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
     --base dev \
     --head codex/issue-<ISSUE_NUMBER>-<short-kebab-title> \
     --title "feat: <issue-title> (#<ISSUE_NUMBER>)" \
     --body "Closes #<ISSUE_NUMBER>"
   ```

9. **Move issue to In Review** on project board:

   ```bash
   gh project item-edit \
     --project-id <PROJECT_ID> \
     --id <ITEM_ID> \
     --field-id <STATUS_FIELD_ID> \
     --single-select-option-id <IN_REVIEW_OPTION_ID>
   ```

10. **Create walkthrough artifact** with what changed, test evidence, and screenshots if UI changed.

11. **Release path**:
   - After issue PRs are merged into `dev`, promote via gated PR `dev -> staging`.
   - Promote to production via PR `staging -> main` only after staging sign-off.
   - Follow `.agent/workflows/release.md` for full release steps (tag + GitHub release).
