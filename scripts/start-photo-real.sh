#!/usr/bin/env bash
# Connect this UI to the existing real ROS executor; do not start or move the arm.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "$HOME/robotics_setup/env.sh"
source "$task_root/install/setup.bash"
source "$HOME/robotics_setup/validation/joint-safety-fix/install/setup.bash"
set -u
export ROS_DOMAIN_ID=0
task_args=(--root "$task_root" --mode real --port "${PEN_HTTP_PORT:-8096}" --profile "$task_root/src/pen_description/config/photo_board.yaml")
if [[ -n "${PEN_CLEARANCE_CHECK:-}" ]]; then
  task_args+=(--clearance-check "$PEN_CLEARANCE_CHECK")
fi
exec python3 -m pen_bringup.web_bridge "${task_args[@]}"
