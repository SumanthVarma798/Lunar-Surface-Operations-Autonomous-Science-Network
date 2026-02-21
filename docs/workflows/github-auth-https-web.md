# GitHub Auth Workflow (HTTPS + Web Device Login)

This workflow logs GitHub CLI in with:
- git protocol: `https`
- git credential auth confirmation: `Y`
- web device flow (browser tab opens)

## Run

```bash
./scripts/gh_auth_https_web.sh
```

## Notes

- Default config dir is `~/.gh_config`.
- Override with:

```bash
GH_CONFIG_DIR=/path/to/gh_config ./scripts/gh_auth_https_web.sh
```

- If your network is down, rerun once DNS/Internet is restored.
