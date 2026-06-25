#!/usr/bin/env bash
#
# run_robot.sh — bring up the NUC container and open a tmux session inside it.
#
# If the a2_ros_nuc container is not running, `docker compose up -d` it;
# otherwise just `docker compose exec` into the existing one. Inside the
# container it creates (or attaches to) a tmux session "a2" with two windows:
#   1: nuc    -> ros2 launch a2_ros nuc.launch.py
#   2: shell  -> 2x2 grid of four panes:
#                pane 1: ros2 launch resple resple_ss26.launch.py map_saving_node:=true (auto-run)
#                pane 2: a2 explore                 (pre-typed, press Enter to start)
#                pane 3: bash ./scripts/save_map.sh (pre-typed, run when mapping is done)
#                pane 4: empty shell (active)
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

# Window 1: nuc — the NUC launch (locomotion, drivers, etc.).
tmux new-session -d -s "$SESSION" -n nuc "bash -i"
tmux send-keys   -t "$SESSION":nuc "ros2 launch a2_ros nuc.launch.py" C-m

# Window 2: shell — 2x2 grid of four panes. send-keys targets the active pane,
# which is always the one just created by the preceding split.
tmux new-window   -t "$SESSION" -n shell "bash -i"
tmux send-keys    -t "$SESSION":shell "ros2 launch resple resple_ss26.launch.py map_saving_node:=true" C-m

# Pre-typed, NOT auto-run: explore moves the robot; save_map only makes sense
# after mapping. Press Enter in each pane when you are ready.
tmux split-window -t "$SESSION":shell "bash -i"
tmux send-keys    -t "$SESSION":shell "a2 explore"

tmux split-window -t "$SESSION":shell "bash -i"
tmux send-keys    -t "$SESSION":shell "bash ./scripts/save_map.sh"

tmux select-layout -t "$SESSION":shell tiled

# Fourth pane: empty shell, left as the active pane.
tmux split-window -t "$SESSION":shell "bash -i"
tmux select-layout -t "$SESSION":shell tiled

# Attach with the empty fourth pane of the shell window active.
tmux select-window -t "$SESSION":shell
exec tmux attach-session -t "$SESSION"
'
