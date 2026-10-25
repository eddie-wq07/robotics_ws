#!/usr/bin/env bash
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "$HOME/robotics_setup/env.sh"
set -u
cd "$task_root"
if [[ ! -x .venv-pen/bin/python ]]; then /usr/bin/python3.10 -m venv .venv-pen; fi
touch .venv-pen/COLCON_IGNORE
env -u PYTHONPATH -u PYTHONHOME .venv-pen/bin/python -m pip install -r requirements-pen.txt
env -u PYTHONPATH -u PYTHONHOME .venv-pen/bin/python -m pip install --no-deps \
  -e src/pen_image_processing -e src/pen_surface_geometry -e src/pen_toolpath
colcon build --packages-select pen_interfaces pen_image_processing pen_surface_geometry \
  pen_toolpath pen_description pen_servo pen_execution pen_bringup flexiv_bringup \
  flexiv_cartesian_controller --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
printf 'Installed. Run %s/scripts/start-pen.sh fake for the fake-hardware browser.\n' "$task_root"
