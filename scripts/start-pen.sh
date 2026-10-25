#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "$task_root/install/setup.bash"
source "$HOME/robotics_setup/env.sh"
set -u
task_mode=${1:-preview}
task_port=${PEN_HTTP_PORT:-8080}
case "$task_mode" in
  preview)
    export ROS_DOMAIN_ID=${PEN_ROS_DOMAIN_ID:-73}
    exec ros2 launch pen_bringup app.launch.py execution_mode:=preview \
      workspace:="$task_root" port:="$task_port"
    ;;
  fake)
    export ROS_DOMAIN_ID=${PEN_ROS_DOMAIN_ID:-73}
    if [[ "$ROS_DOMAIN_ID" == 0 ]]; then printf 'Fake mode requires a separate ROS domain.\n' >&2; exit 2; fi
    exec ros2 launch pen_bringup fake.launch.py workspace:="$task_root" port:="$task_port"
    ;;
  *)
    printf 'Usage: %s [preview|fake]\nPhysical startup requires measured calibration and the operator procedure.\n' "$0" >&2
    exit 2
    ;;
esac
