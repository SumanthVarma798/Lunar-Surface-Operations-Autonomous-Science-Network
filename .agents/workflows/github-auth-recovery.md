# GitHub Auth Recovery Workflow (Generalized)

Use this workflow whenever GitHub CLI auth fails, tokens are invalid, or PR/issue operations return auth/network errors.

## Goals

- Re-authenticate `gh` using:
  - git protocol: `https`
  - confirm git auth prompt: `Y`
  - web/device login flow
- Detect blockers early (missing `gh`, DNS/network issues, broken shell state).
- Keep the user informed with short, explicit progress updates.
- Ensure auth is actually usable for PR/issue/project operations (not just "login complete" output).

## Communication Contract

Before running auth:
1. State what you will do in one sentence.
2. Mention first preflight checks: `gh` installed, network reachable, current auth state.

If blocked:
1. Say exactly what failed (include command + short error).
2. Provide the next concrete fix step.
3. Retry once after the blocker is resolved.

## Step-by-Step Runbook

### 1) Verify `gh` is installed

```bash
command -v gh >/dev/null 2>&1 && gh --version | head -n 1 || echo "gh-missing"
```

If `gh-missing`, ask user before installing.

### 2) Verify internet + DNS for GitHub

```bash
curl -I https://github.com -m 10
```

If this fails with host resolution/connectivity errors, stop auth attempts and ask user to restore network.

### 3) Choose a writable auth config directory

Use a writable config directory explicitly in this environment.

Recommended:

```bash
export GH_CONFIG_DIR=/tmp/gh_config
mkdir -p "$GH_CONFIG_DIR"
```

### 4) Check current auth status

```bash
gh auth status -h github.com
```

### 5) Start auth flow (HTTPS + `Y` + browser)

```bash
printf 'Y\n' | gh auth login -h github.com --git-protocol https --web
```

If a code is shown, communicate it immediately and open:

```bash
open https://github.com/login/device
```

Then wait for user confirmation (`done`).

### 6) Validate auth actually works (mandatory)

Do not trust only `Authentication complete` output.
Immediately run both checks:

```bash
gh auth status -h github.com
gh api user --jq .login
```

If either fails, auth is not usable yet.

### 7) If terminal/session is unstable

Run in a fresh login shell:

```bash
exec "$SHELL" -l
```

Then rerun step 5.

### 8) If browser opener fails

Open manually:

```text
https://github.com/login/device
```

Continue the auth command in terminal.

### 9) Only then run PR operations

Once validation passes:

```bash
gh pr view 68 --json number,title,url,state,headRefName,baseRefName
```

## Common Failure Mapping

- `Could not resolve host: github.com`
  - Cause: DNS/network.
  - Action: Fix connectivity first; do not continue auth.

- `Authentication complete` but `gh auth status` still invalid
  - Cause: token/config not persisted correctly or network/API still failing.
  - Action: set `GH_CONFIG_DIR` to writable location (`/tmp/gh_config`), rerun login, then run both validation checks.

- `mkdir /Users/.../.config/gh: permission denied`
  - Cause: default GH config path not writable in current environment.
  - Action: set `GH_CONFIG_DIR=/tmp/gh_config` and retry.

- `The token ... is invalid`
  - Cause: stale/expired token.
  - Action: rerun web auth workflow with explicit `GH_CONFIG_DIR`.

- `gh: command not found`
  - Cause: CLI missing.
  - Action: ask user approval to install GitHub CLI, then rerun.

## Completion Criteria

All must be true:
1. `gh auth status -h github.com` reports logged-in account.
2. `gh api user --jq .login` succeeds.
3. `gh pr view ...` succeeds for the target repo.
4. User has been told the exact next step if any of the above fail.
