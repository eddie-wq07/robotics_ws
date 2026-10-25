#!/usr/bin/env bash
# Set Flexiv control-box digital output 0 high using the local RDK installation.
# Run only when no other SDK client owns the robot, including on other hosts.
set -e

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    echo "Usage: $0"
    echo "Set control-box DO0 high. Requires Ethernet Remote mode and exclusive RDK access."
    echo "Does not enable the arm, change modes, clear faults, or send motion commands."
    exit 0
fi
if (( $# != 0 )); then
    echo "Unexpected argument. Use --help for usage." >&2
    exit 2
fi

# ROS setup scripts are not compatible with nounset.
source "$HOME/robotics_setup/env.sh"
source "$HOME/robotics-venv/bin/activate"
set -u

if pgrep -f '(^|/|[[:space:]])ros2_control_node([[:space:]]|$)' >/dev/null; then
    echo "A controller manager is running. Use its GPIO controller instead of opening a second RDK connection." >&2
    exit 1
fi

exec timeout --kill-after=3s 20s "$HOME/robotics-venv/bin/python" - <<'PY'
import os
import sys

import flexivrdk

try:
    serial = os.environ["FLEXIV_ROBOT_SN"]
    print(f"Connecting to {serial} to set control-box DO0 HIGH...", flush=True)
    robot = flexivrdk.Robot(serial)
    robot.SetDigitalOutputs({0: True})
except Exception as exc:
    print(f"DO0 HIGH was not confirmed: {exc}", file=sys.stderr, flush=True)
    sys.exit(1)

print("DO0 HIGH command delivered. Physical output voltage has not been measured.", flush=True)
PY
