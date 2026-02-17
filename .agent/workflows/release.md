---
description: End-to-end release workflow using dev -> staging -> main(prod), semantic tags, and GitHub Releases
---

# Release Workflow

Use this workflow for every formal release (`vX.Y.Z`).

## Environment and branch mapping

- `dev` branch -> Dev environment (playground/integration)
- `staging` branch -> Staging environment (production replica)
- `main` branch -> Production environment

Promotion chain is always:

1. `dev -> staging`
2. `staging -> main`

`prod` must always be a replica of what passed in `staging`.

## Inputs required

- Release version (for example `v1.0.0`, `v1.1.0`, `v2.0.0`)
- Scope summary (features/fixes/docs)
- Target milestone and included issues
- Go/no-go approvers for staging and prod gates
- Release checklist file copied from `.agent/workflows/release-checklist-template.md`

## Mandatory guardrails

- No direct feature PRs to `staging` or `main`
- No force pushes to `dev`, `staging`, or `main`
- Branch protection enabled with required checks on `staging` and `main`
- All PR merges should be non-interactive and traceable
- Treat `master` references as `main` in this repository

## 0. One-time branch setup (if missing)

```bash
git fetch origin
git checkout main
git pull --ff-only origin main

git checkout -b staging origin/main
git push -u origin staging

git checkout -b dev origin/staging
git push -u origin dev
```

If branches already exist, do not recreate them.

## 1. Pre-flight sync

```bash
git fetch origin
git checkout main
git pull --ff-only origin main
git checkout staging
git pull --ff-only origin staging
git checkout dev
git pull --ff-only origin dev
```

Initialize release checklist record:

```bash
mkdir -p docs/releases
cp .agent/workflows/release-checklist-template.md docs/releases/<VERSION>-checklist.md
```

## 2. Release readiness on dev

- Confirm all intended feature PRs are merged into `dev`
- Confirm `dev` CI is green
- Confirm release milestone has no unresolved P0/P1 blockers
- Confirm working tree clean:

```bash
git status -sb
```

## 3. Build release scope notes

```bash
LAST_TAG=$(git describe --tags --abbrev=0 2>/dev/null || echo "")
if [ -n "$LAST_TAG" ]; then
  git log --oneline "$LAST_TAG"..origin/dev
else
  git log --oneline origin/dev -n 100
fi
```

Capture:

- Highlights
- Fixes
- Docs/process changes
- Breaking changes and migrations

## 4. Gate 1 PR: dev -> staging

Prepare PR body:

```bash
cat > /tmp/release_stage_pr.md <<'PRBODY'
## Purpose
Promote validated development work into staging for production-like verification.

## Included scope
- <feature/fix/docs summary>

## Validation plan
- [ ] CI green on staging PR
- [ ] Manual smoke on staging environment
- [ ] No blocker defects

## Roll-forward / rollback
- Roll-forward fixes land in dev and are re-promoted
- Rollback by reverting staging merge if needed
PRBODY
```

Open PR:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr create \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --base staging \
  --head dev \
  --title "release-candidate: <VERSION> to staging" \
  --body-file /tmp/release_stage_pr.md
```

## 5. Staging validation gate

On the `dev -> staging` PR:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr view <STAGE_PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --json mergeable,mergeStateStatus,statusCheckRollup
```

Required to pass before merge:

- CI checks green
- Staging deploy successful
- Manual smoke checklist signed off
- No unresolved release blockers

Merge gate PR:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr merge <STAGE_PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --merge --delete-branch=false
```

## 6. Freeze release candidate from staging

```bash
git fetch origin
git checkout staging
git pull --ff-only origin staging
STAGING_SHA=$(git rev-parse --short HEAD)
echo "$STAGING_SHA"
```

Use this SHA as release candidate fingerprint.

## 7. Gate 2 PR: staging -> main (production)

Prepare prod PR body:

```bash
cat > /tmp/release_prod_pr.md <<'PRBODY'
## Purpose
Promote staging-validated release candidate to production.

## Candidate
- Staging SHA: <STAGING_SHA>
- Version: <VERSION>

## Production gate checklist
- [ ] Staging validation completed
- [ ] Required status checks green
- [ ] Release notes prepared
- [ ] Rollback owner assigned
PRBODY
```

Open PR:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr create \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --base main \
  --head staging \
  --title "release: <VERSION> to production" \
  --body-file /tmp/release_prod_pr.md
```

Verify and merge:

```bash
export GH_CONFIG_DIR=/Users/varma/.gh_config
gh pr view <PROD_PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --json mergeable,mergeStateStatus,statusCheckRollup

gh pr merge <PROD_PR_NUMBER> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --merge --delete-branch=false
```

## 8. Tag production release

```bash
git fetch origin
git checkout main
git pull --ff-only origin main
RELEASE_SHA=$(git rev-parse --short HEAD)

git tag -a <VERSION> -m "LSOAS <VERSION> release" "$RELEASE_SHA"
git push origin <VERSION>
```

## 9. Publish GitHub release

```bash
cat > /tmp/release_notes.md <<'NOTES'
# LSOAS <VERSION>

## Highlights
- <highlight 1>
- <highlight 2>

## Scope
- <scope summary>

## Deployment path
- Promoted via dev -> staging -> main
NOTES

export GH_CONFIG_DIR=/Users/varma/.gh_config
gh release create <VERSION> \
  -R SumanthVarma798/Lunar-Surface-Operations-Autonomous-Science-Network \
  --target main \
  --title "LSOAS <VERSION>" \
  --notes-file /tmp/release_notes.md
```

## 10. Roadmap and milestone updates

After production release:

1. Close delivered issues in completed milestone
2. Close completed milestone(s)
3. Create/activate next milestone and epic/issues
4. Update repo artifacts:
   - `README.md`
   - `docs/roadmap.md`
   - `docs/chandrayaan_v2_migration.md`
   - `.github/scripts/generate_project_structure.py`
   - `github_project_structure.json`

## 11. Post-release verification checklist

- [ ] `main` equals released production state
- [ ] `staging` equals the promoted release candidate
- [ ] Tag exists locally and on remote
- [ ] GitHub release published
- [ ] Roadmap state reflects release closure and next phase

## 12. Rollback strategy

If production issue is discovered:

1. Revert the `staging -> main` merge commit on `main`
2. Deploy reverted `main`
3. Merge/cherry-pick the same revert into `staging` and `dev`
4. Open follow-up fix issue and hotfix PR chain

## 13. Hotfix path

For urgent production fixes:

1. Branch `codex/hotfix-<short-name>` from `main`
2. PR to `main`, validate, and merge
3. Forward-merge hotfix to `staging`, then to `dev`
4. Tag patch release (`vX.Y.(Z+1)`) and publish release notes

## Example timeline

- Feature PRs: `codex/* -> dev`
- Release candidate: `dev -> staging`
- Production release: `staging -> main`
- Tag + GitHub release + roadmap closure
