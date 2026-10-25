#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "$task_root/install/setup.bash"
source "$HOME/robotics_setup/env.sh"
set -u
exec python3 -m pen_bringup.web_bridge --root "$task_root" \
  --mode preview --no-ros --port "${PEN_HTTP_PORT:-8095}" \
  --profile "$task_root/src/pen_description/config/tylenol.yaml"
