# `.agent` Directory Guide

This folder defines the repository's shared agent workflows.

## Contents

- `workflows/`: canonical runbooks for branch/PR flow, implementation, release, and related tasks.
- `artifacts/`: local scratch output (plans, walkthrough drafts). This path is intentionally ignored in git.

## Contributor expectation

When using an agent on this repo (including forks), treat `workflows/` as the default process documentation and `AGENTS.md` as the entrypoint.
