---
description: Implement a GitHub issue from the LSOAS project board
---

# Implement Issue

This workflow fetches a GitHub issue by number and implements it end-to-end.

## Steps

// turbo-all

1. **Fetch the issue** from GitHub using the CLI:

   ```
   export GH_CONFIG_DIR=/Users/varma/.gh_config && gh issue view <ISSUE_NUMBER> -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network --json title,body,labels,milestone
   ```

   Replace `<ISSUE_NUMBER>` with the issue number provided by the user (e.g. `3`, `21`).

2. **Read the issue body** carefully. Extract:
   - The task description and goals
   - The list of files to modify or create
   - All acceptance criteria checkboxes

3. **Create a feature branch**:

   ```
   git checkout -b feature/<short-kebab-description-from-issue-title>
   ```

4. **Plan the implementation** by creating an implementation plan artifact. Present it to the user for approval before writing code.

5. **Implement the changes** file by file, following the acceptance criteria as a checklist. Write clean, well-documented code.

6. **Verify the implementation**:
   - Run any relevant tests or linters
   - If the issue involves the web dashboard, open http://localhost:8080 in the browser and visually verify
   - Check each acceptance criterion from the issue body

7. **Commit and push**:

   ```
   git add -A && git commit -m "feat: <issue-title> (#<ISSUE_NUMBER>)"
   git push origin HEAD
   ```

8. **Create a pull request**:

   ```
   export GH_CONFIG_DIR=/Users/varma/.gh_config && gh pr create -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network --title "<issue-title>" --body "Closes #<ISSUE_NUMBER>" --label "<comma-separated-labels-from-issue>"
   ```

9. **Move the issue on the project board** — update its status to "In review":

   ```
   export GH_CONFIG_DIR=/Users/varma/.gh_config && gh project item-edit --project-id PVT_kwHOA2WGPc4BPOeG --id <ITEM_ID> --field-id <STATUS_FIELD_ID> --single-select-option-id <IN_REVIEW_OPTION_ID>
   ```

10. **Create a walkthrough** artifact summarizing what was implemented, what was tested, and any screenshots or recordings captured.
