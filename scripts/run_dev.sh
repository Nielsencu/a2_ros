#!/usr/bin/env bash
#
# run_dev.sh — bring up the dev container and drop into a shell inside it.
#
# If the a2_ros_dev container is not running, `docker compose up -d` it;
# otherwise just `docker compose exec` into the existing one. From the shell
# inside you can launch a workload, e.g.:
#   ./scripts/run_sim.sh    # MuJoCo sim + foxglove + teleop
#   ./scripts/run_bag.sh    # foxglove + dlio + bag playback
#
# Usage: ./scripts/run_dev.sh

set -euo pipefail

# Run from the repo root so docker compose finds compose.yaml.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

SERVICE="a2_ros_dev"

# Start the container if it isn't already running; otherwise reuse it.
if [ -z "$(docker compose ps -q --status running "$SERVICE" 2>/dev/null)" ]; then
  echo "[run_dev] '$SERVICE' is not running — starting it."
  docker compose up -d "$SERVICE"
else
  echo "[run_dev] '$SERVICE' is already running — exec'ing in."
fi

# Drop into an interactive shell inside the container.
exec docker compose exec "$SERVICE" bash
