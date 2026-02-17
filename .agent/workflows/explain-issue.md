---
description: Explain a GitHub issue in detail — what it involves, why it matters, and how to approach it
---

# Explain Issue

This workflow fetches a GitHub issue and provides a comprehensive explanation.

Prerequisite: run `gh auth login` once on your machine before using `gh` commands.

## Steps

// turbo-all

1. **Fetch the issue** from GitHub:

   ```
   gh issue view <ISSUE_NUMBER> -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network --json title,body,labels,milestone,assignees,state
   ```

   Replace `<ISSUE_NUMBER>` with the issue number provided by the user.

2. **Read the relevant source files** listed in the issue body to understand the current state of the code.

3. **Provide a structured explanation** covering:
   - **Summary**: One-paragraph plain-English explanation of what this issue is about
   - **Why it matters**: How this fits into the overall LSOAS roadmap and which phase/milestone it belongs to
   - **Scope of changes**: Which files will be created or modified, and a brief description of each change
   - **Technical approach**: How to implement it — key algorithms, libraries, patterns, or architecture decisions
   - **Dependencies**: What must be completed before this issue can start (other issues, libraries, data)
   - **Estimated effort**: Based on the complexity and duration labels
   - **Release fit**: Whether this issue should land in the current release train or a later one, and why
   - **Integration path**: Recommended branch/PR target (`codex/*` -> `dev`; release gates `dev` -> `staging` -> `main`)
   - **Acceptance criteria**: Restate each checkbox item and explain what "done" looks like for each
   - **Risks & gotchas**: Potential challenges or tricky parts to watch out for
   - **Learning resources**: If the issue touches unfamiliar concepts (e.g. Three.js, orbital mechanics), suggest specific tutorials or docs

4. **If the issue references other issues** (e.g. an EPIC or dependency), fetch those too and explain the relationship.
