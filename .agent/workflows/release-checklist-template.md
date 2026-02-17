# Release Checklist Template

Use this template for each release run.

## Metadata

- Version:
- Date (UTC):
- Release lead:
- Staging approver:
- Production approver:
- Milestone:

## Scope

- Included PRs:
- Included issues:
- Excluded/deferred items:
- Breaking changes:
- Migration notes:

## Gate 1: dev -> staging

- [ ] PR created (`dev` -> `staging`):
- [ ] All required CI checks green
- [ ] Staging deploy successful
- [ ] Manual smoke passed
- [ ] No release blockers
- [ ] PR merged
- [ ] Staging SHA recorded:

## Gate 2: staging -> main

- [ ] PR created (`staging` -> `main`):
- [ ] All required CI checks green
- [ ] Final go/no-go approval recorded
- [ ] PR merged
- [ ] Production SHA recorded:

## Tag and Release

- [ ] Annotated tag created:
- [ ] Tag pushed to origin
- [ ] GitHub Release published URL:

## Post-release

- [ ] Delivered issues closed
- [ ] Completed milestone closed
- [ ] Next milestone/epic created or activated
- [ ] README/roadmap docs updated
- [ ] Incident watch window completed

## Rollback Plan (if needed)

- Main revert owner:
- Staging revert owner:
- Dev forward-fix owner:
- Comms channel:

## Notes

- Lessons learned:
- Follow-up actions:
