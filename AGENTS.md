# Repository Agent Instructions

These instructions apply to the entire repository.

## Default workflow for agents

1. Start with `.agent/workflows/branch-and-pr.md` for any feature, fix, docs, or cleanup change.
2. If implementing a GitHub issue, follow `.agent/workflows/implement-issue.md`.
3. For release and promotion work (`dev -> staging -> main`), follow `.agent/workflows/release.md`.

## What should stay in git

- Keep `.agent/workflows/` committed. It is the shared, repo-specific playbook for contributors and forks.
- Keep release templates under `.agent/workflows/` committed.

## What should stay local

- `.agent/artifacts/` is for transient implementation plans and walkthrough notes.
- Do not commit generated issue-specific artifacts unless explicitly requested by maintainers.
