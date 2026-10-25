#!/usr/bin/env bash
# Offline photo-board preview: no ROS bridge or physical robot connection.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mkdir -p "$task_root/assets/inputs"
for photo_asset in eddiemom.png eddiemom.png.strokes.json; do
  if [[ ! -f "$task_root/assets/inputs/$photo_asset" ]]; then
    cp "$task_root/src/pen_description/assets/photo_board/$photo_asset" "$task_root/assets/inputs/$photo_asset"
  fi
done
set +u
source "$HOME/robotics_setup/env.sh"
source "$task_root/install/setup.bash"
source "$HOME/robotics_setup/validation/joint-safety-fix/install/setup.bash"
set -u
exec python3 -m pen_bringup.web_bridge --root "$task_root" \
  --mode preview --no-ros --port "${PEN_HTTP_PORT:-8096}" \
  --profile "$task_root/src/pen_description/config/photo_board.yaml"
