#!/usr/bin/env bash
set -euo pipefail

IMAGE="ros:humble"
NAME="ros-humble-dev"
WS_HOST="$(cd "$(dirname "$0")/.." && pwd)"

# Start container if it isn't running
if ! docker ps --format '{{.Names}}' | grep -q "^${NAME}$"; then
  docker rm -f "${NAME}" >/dev/null 2>&1 || true
  docker run -dit \
    --name "${NAME}" \
    -v "${WS_HOST}:/workspace" \
    -w /workspace/lunar_ops/rover_ws \
    "${IMAGE}" \
    bash
fi

# Open a shell in it
docker exec -it "${NAME}" bash -lc "source /opt/ros/humble/setup.bash && exec bash"
