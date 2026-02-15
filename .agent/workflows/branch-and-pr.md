---
description: How to create a feature branch, commit changes, and raise a PR
---

# Feature Branch & PR Workflow

Follow this process for **every new feature or change**.

## 1. Start from latest main

// turbo

```bash
git checkout main && git pull origin main
```

## 2. Create a feature branch

Use the naming convention: `feature/<short-description>`, `fix/<short-description>`, or `chore/<short-description>`

```bash
git checkout -b feature/<branch-name>
```

## 3. Make changes

Implement the feature, fix, or chore.

## 4. Stage and commit changes

Use **conventional commit** messages:

- `feat:` for new features
- `fix:` for bug fixes
- `chore:` for maintenance/cleanup
- `docs:` for documentation
- `refactor:` for code restructuring

```bash
git add <files>
git commit -m "feat: short description of what changed"
```

## 5. Push the branch

```bash
git push -u origin feature/<branch-name>
```

## 6. Create a Pull Request

If `gh` CLI is authenticated:

```bash
gh pr create --title "feat: Title" --body "Description" --base main
```

Otherwise, open the PR URL printed by git push:

```
https://github.com/SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network/pull/new/<branch-name>
```

## 7. After PR is merged

// turbo

```bash
git checkout main && git pull origin main
```

Then delete the local branch:
// turbo

```bash
git branch -d feature/<branch-name>
```

## Notes

- Always commit working code — don't commit broken states
- Keep commits atomic — one logical change per commit
- Write descriptive PR bodies explaining what changed and why
- Build/install/log artifacts are gitignored and should never be committed
