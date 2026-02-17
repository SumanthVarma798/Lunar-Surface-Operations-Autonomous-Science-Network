---
description: End-to-end release workflow for LSOAS using develop -> main, semantic tags, and GitHub Releases
---

# Release Workflow

Use this workflow for every formal release (`vX.Y.Z`).

## Release model

- Integration branch: `develop`
- Release branch: `main`
- Promotion mechanism: PR from `develop` to `main`
- Release artifact: annotated git tag + GitHub Release

## Inputs required

- Release version (for example `v1.0.0`, `v1.1.0`, `v2.0.0`)
- Release scope summary (features/fixes/docs)
- Milestone/issues to close
- Decision: merge strategy (`merge` default)

## Guardrails

- Do not commit directly to `main` for normal work.
- Do not force-push tags or release branches.
- Use `--ff-only` pulls to avoid accidental local merge commits.
- Treat `master` references as `main` in this repository.

## 1. Pre-flight sync

```bash
git fetch origin
git checkout main
git pull --ff-only origin main
git checkout develop
git pull --ff-only origin develop
```

## 2. Validate release readiness

1. Ensure all planned feature PRs are merged into `develop`.
2. Verify CI on latest `develop` is green.
3. Confirm no critical open blockers in the release milestone.
4. Confirm working tree is clean:

```bash
git status -sb
```

## 3. Build release scope notes

Find the previous release tag and summarize changes since then:

```bash
LAST_TAG=$(git describe --tags --abbrev=0 2>/dev/null || echo "")
if [ -n "$LAST_TAG" ]; then
  git log --oneline "$LAST_TAG"..origin/develop
else
  git log --oneline origin/develop -n 100
fi
```

Capture:

- Key features
- Key fixes
- Docs/roadmap updates
- Breaking changes (if any)

## 4. Open release PR (develop -> main)

Create a release PR body file:

```bash
cat > /tmp/release_pr.md <<'PRBODY'
## Problem
<why this release is needed now>

## What changed
- <item 1>
- <item 2>

## Why this design
- <reasoning>

## Testing evidence
- <CI/jobs/manual evidence>

## Migration notes
- <breaking changes or "none">

## Superseded issue mapping (old -> new)
- <if applicable>
PRBODY
```

Open PR:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr create \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --base main \
  --head develop \
  --title "release: <VERSION>" \
  --body-file /tmp/release_pr.md
```

## 5. Gate and merge release PR

Check mergeability and status checks:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr view <PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --json mergeable,mergeStateStatus,statusCheckRollup
```

When all required checks pass and approvals are done:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr merge <PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --merge --delete-branch=false
```

## 6. Sync local main to merged release

```bash
git fetch origin
git checkout main
git pull --ff-only origin main
RELEASE_SHA=$(git rev-parse --short HEAD)
echo "$RELEASE_SHA"
```

## 7. Create annotated release tag

```bash
git tag -a <VERSION> -m "LSOAS <VERSION> release" "$RELEASE_SHA"
git push origin <VERSION>
```

Tag naming convention: semantic version with `v` prefix.

## 8. Publish GitHub Release

Create release notes file:

```bash
cat > /tmp/release_notes.md <<'NOTES'
# LSOAS <VERSION>

## Highlights
- <highlight 1>
- <highlight 2>

## Scope
- <scope summary>

## Roadmap status
- <phase updates>
NOTES
```

Publish:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh release create <VERSION> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --target main \
  --title "LSOAS <VERSION>" \
  --notes-file /tmp/release_notes.md
```

## 9. Update roadmap and milestones

1. Close delivered issues in the completed milestone.
2. Close completed milestone(s).
3. Create/activate next milestone and roadmap epic/issues.
4. Update repository roadmap artifacts:
   - `README.md`
   - `docs/roadmap.md`
   - `docs/chandrayaan_v2_migration.md`
   - `.github/scripts/generate_project_structure.py`
   - `github_project_structure.json`

## 10. Post-release verification checklist

- [ ] `main` and `origin/main` are aligned
- [ ] Release PR is merged
- [ ] Tag exists locally and remotely
- [ ] GitHub release page is published
- [ ] Roadmap issues/milestones reflect new phase state
- [ ] Docs mention latest release baseline

## 11. Hotfix path (if required)

If urgent fix is required after release:

1. Create `codex/hotfix-<short-name>` from `main`.
2. Open PR to `main`.
3. After merge, cherry-pick or merge hotfix into `develop` immediately.
4. Tag patch release (`vX.Y.(Z+1)`) and publish release notes.

## Example timeline

- `develop` accumulates feature PRs.
- Release PR `develop -> main` opened and reviewed.
- Merge PR, tag `vX.Y.Z`, publish GitHub Release.
- Close completed milestone; open next phase milestone.
