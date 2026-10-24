#!/bin/bash
# Launch only Flexiv's mock backend. No real-robot mode in this script.
set -eo pipefail
task_root=$(cd "$(dirname "$0")" && pwd)
if [ "$#" -ne 1 ]; then echo "Usage: ./rviz.sh PATH/TO/job.json" >&2; exit 2; fi
job_file=$(python3 -c 'import pathlib,sys; print(pathlib.Path(sys.argv[1]).resolve())' "$1")
[ -f "$job_file" ] || { echo "Job not found: $job_file" >&2; exit 1; }
if [ ! -f /opt/ros/humble/setup.bash ]; then
  echo "RViz requires the Ubuntu 22.04 / ROS 2 Humble PC. See docs/ros.md." >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
flexiv_setup=${FLEXIV_WS:-$HOME/flexiv_ros2_ws}/install/setup.bash
[ -f "$flexiv_setup" ] || { echo "Missing Flexiv workspace: $flexiv_setup. See docs/ros.md." >&2; exit 1; }
source "$flexiv_setup"
export ROS_DOMAIN_ID=73
export ROS_LOCALHOST_ONLY=1
ros2 launch flexiv_bringup rizon_moveit.launch.py \
  robot_type:=Rizon4s robot_sn:=Rizon4s-000000 use_fake_hardware:=true \
  load_gripper:=false start_servo:=false &
flexiv_pid=$!
trap 'kill "$flexiv_pid" 2>/dev/null || true; wait "$flexiv_pid" 2>/dev/null || true' EXIT
trap 'exit 130' INT TERM
/usr/bin/python3 "$task_root/ros/preview.py" "$job_file"
