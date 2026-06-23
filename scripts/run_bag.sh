#!/usr/bin/env bash
#
# run_bag.sh — spawn a tmux session with three windows:
#   1: foxglove  -> a2 foxglove
#   2: dlio      -> a2 dlio
#   3: bag       -> empty, split into two panes (e.g. ros2 bag play + spare shell)
#
# Usage: ./scripts/run_bag.sh
 
set -euo pipefail
 
SESSION="a2"
 
# If the session already exists, just attach to it instead of duplicating.
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists — attaching."
  exec tmux attach-session -t "$SESSION"
fi
 
tmux new-session -d -s "$SESSION" -n foxglove 'a2 foxglove' \; \
  set-option -g remain-on-exit off \; \
  new-window -n dlio a2 dlio  \; \
  new-window -n bag  \; \
  split-window -h -t "$SESSION":bag  \; \
  send-keys -t "$SESSION":bag.0 'ros2 service call /save_pcd direct_lidar_inertial_odometry/srv/SavePCD "{leaf_size: 0.10, save_path: \"$MAP_DIR\"}"' \; \
  select-pane -t "$SESSION":bag.1 \; \
  attach-session -t "$SESSION"