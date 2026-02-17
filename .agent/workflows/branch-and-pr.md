---
description: How to create a feature branch, commit changes, and raise PRs with dev, staging, and production branches
---

# Feature Branch and PR Workflow

Follow this process for every feature, fix, or docs change.

Prerequisite: run `gh auth login` once on your machine before using `gh` commands.

## Branch and environment model

- `dev`: integration branch for daily development (deploys to Dev environment)
- `staging`: production replica and release gate (deploys to Staging)
- `main`: production branch (deploys to Prod)
- `codex/*`: short-lived feature branches created from `dev`

`develop` is legacy and should not receive new feature PRs.

Normal feature work must not target `staging` or `main` directly.

## 1. Sync local branches

// turbo

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
```

## 2. Create a working branch from dev

Use `codex/<short-kebab-description>`.

```bash
git checkout dev
git pull --ff-only origin dev
git checkout -b codex/<branch-name>
```

## 3. Implement and verify

- Make required changes.
- Run relevant tests/lint checks before committing.
- Keep commits small and reviewable.

## 4. Stage and commit

Use conventional commits:

- `feat:` new functionality
- `fix:` bug fix
- `docs:` documentation changes
- `refactor:` internal restructuring
- `chore:` maintenance

```bash
git add <files>
git commit -m "feat: short description"
```

## 5. Push branch

```bash
git push -u origin codex/<branch-name>
```

## 6. Open PR to dev

```bash
gh auth status || gh auth login
gh pr create \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --base dev \
  --head codex/<branch-name> \
  --title "feat: short PR title" \
  --body "Summary of what changed and why"
```

## 7. Promotion flow

- Merge approved feature PRs into `dev`.
- Batch and gate releases with PR `dev -> staging`.
- Promote only validated staging builds with PR `staging -> main`.

Use `.agent/workflows/release.md` for full release steps.

## 8. After merge

// turbo

```bash
git checkout dev
git pull --ff-only origin dev
git checkout staging
git pull --ff-only origin staging
git checkout main
git pull --ff-only origin main
git branch -d codex/<branch-name>
```

## Notes

- Never force-push to `dev`, `staging`, or `main`.
- Never commit directly to `main` for normal work.
- If a hotfix lands on `main`, forward-merge it into `staging` and then `dev` immediately.
