#!/bin/bash
# detection_postprocess.sh — launch the full camera detection + logging pipeline.
#
# Starts these as concurrent background processes (in startup order):
#   1. image_transport republish — decompress /camera/image_raw/compressed -> /camera/image_raw
#   2. rectify_node.py (OpenCV)   — /camera/image_raw (+camera_info) -> /camera/image_rect
#   3. object_logger.py           — log detected objects to CSV in the map frame
#   4. a2 resple                  — RESPLE state estimation (add --rviz to visualize)
#   5. object_detection           — YOLO object detection launch
#
# Usage:
#   ./detection_postprocess.sh [--rviz]
#     --rviz   run RESPLE with RViz (a2 resple --rviz)
#
# Ctrl-C stops the entire pipeline.

set -u

# ---- args ----
RVIZ=false
for arg in "$@"; do
    case "$arg" in
        --rviz) RVIZ=true ;;
        -h|--help) sed -n '2,15p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Unknown argument: $arg (use --rviz)"; exit 1 ;;
    esac
done

SCRIPT_DIR="$(cd "$(dirname "$(realpath "${BASH_SOURCE[0]}")")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Unique, timestamped CSV in the postprocess/ folder.
POSTPROCESS_DIR="$WORKSPACE_DIR/postprocess"
mkdir -p "$POSTPROCESS_DIR"
CSV_PATH="$POSTPROCESS_DIR/detected_objects_$(date +%Y%m%d_%H%M%S).csv"

# ---- environment (no-op if the shell is already sourced) ----
# ROS/colcon setup files reference unset vars (COLCON_TRACE, etc.), so disable
# nounset while sourcing them, then restore it.
set +u
if ! command -v ros2 >/dev/null 2>&1; then
    # shellcheck disable=SC1091
    source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
fi
if [ -f /a2_ros_ws/install/setup.bash ]; then
    # shellcheck disable=SC1091
    source /a2_ros_ws/install/setup.bash
fi
set -u

# ---- process management ----
PIDS=()       # process-group leaders, one per launched component
LABELS=()

cleanup() {
    echo
    echo "[detection_postprocess] stopping pipeline..."
    # SIGINT each whole process group (negative PID), in reverse startup order,
    # so ros2 launch / a2 children get torn down too.
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        kill -INT -- "-${PIDS[i]}" 2>/dev/null
    done
    sleep 2
    for ((i=${#PIDS[@]}-1; i>=0; i--)); do
        kill -KILL -- "-${PIDS[i]}" 2>/dev/null
    done
    wait 2>/dev/null
    echo "[detection_postprocess] done."
    exit 0
}
trap cleanup INT TERM

launch() {  # launch <label> <cmd...>
    local label="$1"; shift
    echo "[detection_postprocess] starting: $label"
    setsid "$@" &           # new session -> PGID == $!
    PIDS+=("$!")
    LABELS+=("$label")
    sleep 2
}

# ---- pipeline ----
launch "republish (decompress)" \
    ros2 run image_transport republish --ros-args \
        -p in_transport:=compressed -p out_transport:=raw -p use_sim_time:=true \
        -r in/compressed:=/camera/image_raw/compressed -r out:=/camera/image_raw

launch "rectify_node (OpenCV)" \
    python3 "$SCRIPT_DIR/rectify_node.py" --ros-args \
        -r image:=/camera/image_raw -r camera_info:=/camera/camera_info \
        -r image_rect:=/camera/image_rect -p use_sim_time:=true

launch "object_logger" \
    python3 "$SCRIPT_DIR/object_logger.py" --ros-args \
        -p use_sim_time:=true -p csv_path:="$CSV_PATH"

if $RVIZ; then
    launch "a2 resple --rviz" a2 resple --rviz
else
    launch "a2 resple" a2 resple
fi

launch "object_detection" \
    ros2 launch object_detection object_detection.launch.py

echo "[detection_postprocess] all components started (${#PIDS[@]}). Press Ctrl-C to stop."
echo "[detection_postprocess] CSV: $CSV_PATH"

# Wait for any component to exit; keep running until Ctrl-C.
wait
