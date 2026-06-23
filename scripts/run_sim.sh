#!/usr/bin/env bash
#
# run_sim.sh — spawn a tmux session with three windows:
#   1: sim       -> a2 sim
#   2: foxglove  -> a2 foxglove
#   3: shell     -> stand + unlock + walk once the sim is up
#
# Usage: ./scripts/run_sim.sh
 
set -euo pipefail
 
SESSION="a2"
 
# If the session already exists, just attach to it instead of duplicating.
if tmux has-session -t "$SESSION" 2>/dev/null; then
  echo "Session '$SESSION' already exists — attaching."
  exec tmux attach-session -t "$SESSION"
fi
 
tmux new-session -d -s "$SESSION" -n sim 'a2 sim' \; \
  set-option -g remain-on-exit off \; \
  new-window -n foxglove 'a2 foxglove' \; \
  new-window -n shell 'sleep 3 && a2 stand && a2 unlock && a2 walk' \; \
  attach-session -t "$SESSION"