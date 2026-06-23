#!/usr/bin/env bash
#
# run_robot.sh — bring up the NUC container and open a tmux session inside it.
#
# If the a2_ros_nuc container is not running, `docker compose up -d` it;
# otherwise just `docker compose exec` into the existing one. Inside the
# container it creates (or attaches to) a tmux session "a2" with three windows:
#   1: nuc       -> ros2 launch a2_ros nuc.launch.py
#   2: foxglove  -> a2 foxglove
#   3: shell     -> empty shell, split into two panes
#
# Usage: ./scripts/run_robot.sh

set -euo pipefail

# Run from the repo root so docker compose finds compose.yaml.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

SERVICE="a2_ros_nuc"

# Start the container if it isn't already running; otherwise reuse it.
if [ -z "$(docker compose ps -q --status running "$SERVICE" 2>/dev/null)" ]; then
  echo "[run_robot] '$SERVICE' is not running — starting it."
  docker compose up -d "$SERVICE"
else
  echo "[run_robot] '$SERVICE' is already running — exec'ing in."
fi

# Inside the container: create the tmux session (or attach if it already exists).
# Each pane runs an interactive bash ("bash -i") so ~/.bashrc is sourced and the
# ROS environment + the `a2` CLI are available; the commands are then typed into
# those shells, so they stay alive (and re-runnable) after Ctrl-C.
exec docker compose exec "$SERVICE" bash -c '
set -eu
SESSION=a2

if tmux has-session -t "$SESSION" 2>/dev/null; then
  exec tmux attach-session -t "$SESSION"
fi

tmux new-session  -d -s "$SESSION" -n nuc      "bash -i"
tmux new-window      -t "$SESSION"  -n foxglove "bash -i"
tmux new-window      -t "$SESSION"  -n shell    "bash -i"
tmux split-window -h -t "$SESSION":shell        "bash -i"

tmux send-keys -t "$SESSION":nuc      "ros2 launch a2_ros nuc.launch.py" C-m
tmux send-keys -t "$SESSION":foxglove "a2 foxglove" C-m

tmux select-window -t "$SESSION":nuc
exec tmux attach-session -t "$SESSION"
'
