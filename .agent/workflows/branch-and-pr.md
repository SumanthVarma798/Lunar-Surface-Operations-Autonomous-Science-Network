---
description: How to create a feature branch, commit changes, and raise a PR using main + develop
---

# Feature Branch and PR Workflow

Follow this process for every feature, fix, or docs change.

## Branch model

- `main`: production/release branch
- `develop`: integration branch for day-to-day work
- `codex/*`: feature branches created from `develop`

Do not open normal feature PRs directly into `main`.

## 1. Sync local branches

// turbo

```bash
git fetch origin
git checkout main
git pull --ff-only origin main
```

If `develop` does not exist locally, create it from `origin/main`:

```bash
git checkout -b develop origin/main
git push -u origin develop
```

If `develop` already exists, sync it with:

```bash
git checkout develop
git pull --ff-only origin develop
```

## 2. Create a working branch from develop

Use `codex/<short-kebab-description>`.

```bash
git checkout develop
git pull --ff-only origin develop
git checkout -b codex/<branch-name>
```

## 3. Implement and verify

- Make the required changes.
- Run relevant tests/lint checks before committing.
- Keep commits atomic.

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

## 6. Open PR to develop

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config

gh pr create \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --base develop \
  --head codex/<branch-name> \
  --title "feat: short PR title" \
  --body "Summary of what changed and why"
```

## 7. After merge

// turbo

```bash
git checkout develop
git pull --ff-only origin develop
git checkout main
git pull --ff-only origin main
git branch -d codex/<branch-name>
```

## 8. Release promotion

When a release is ready, open `develop -> main` release PR.

Use:

- `.agent/workflows/release.md`

## Notes

- Never force-push to `main` or `develop`.
- Never commit directly to `main` unless doing an approved hotfix.
- If a hotfix goes to `main`, merge it back into `develop` immediately.
