---
description: Compare two GitHub issues — scope, complexity, overlap, and recommended order
---

# Compare Issues

This workflow fetches two GitHub issues and provides a side-by-side comparison.

## Steps

// turbo-all

1. **Fetch both issues** from GitHub:

   ```
   export GH_CONFIG_DIR=/Users/varma/.gh_config && gh issue view <ISSUE_A> -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network --json title,body,labels,milestone
   ```

   ```
   export GH_CONFIG_DIR=/Users/varma/.gh_config && gh issue view <ISSUE_B> -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network --json title,body,labels,milestone
   ```

   Replace `<ISSUE_A>` and `<ISSUE_B>` with the two issue numbers provided by the user.

2. **Read the relevant source files** referenced in both issues to understand the current codebase state.

3. **Present a comparison** in a structured format covering:
   - **At a glance**: Side-by-side table with title, phase, milestone, priority, complexity, estimated duration, category, and number of acceptance criteria
   - **Scope overlap**: Files touched by both issues — highlight shared files and explain potential merge conflicts or coordination needs
   - **Dependency relationship**: Does one issue depend on the other? Should one be completed first? Are they independent and parallelizable?
   - **Complexity comparison**: Which is harder and why (new concepts, more files, tighter integration, performance requirements)
   - **Effort comparison**: Estimated time for each, and whether doing one first makes the other easier
   - **Release impact**: Whether each issue targets current release baseline behavior or next release scope, and any migration risk
   - **Recommended order**: Which issue should be tackled first and why, considering dependencies, risk reduction, and learning curve
   - **Combined implementation notes**: If both could be done together in one branch, explain whether that's advisable or not and why
   - **Branch/PR recommendation**: Suggested branch split and PR order using `codex/*` branches into `dev`, with promotion gates `dev -> staging -> main`
