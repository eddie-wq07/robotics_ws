#!/usr/bin/env bash
# Connect the UI to the existing ROS executor. No robot startup or motion.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
set +u
source "$HOME/robotics_setup/env.sh"
source "$task_root/install/setup.bash"
if [[ -f "$HOME/robotics_setup/validation/joint-safety-fix/install/setup.bash" ]]; then
  source "$HOME/robotics_setup/validation/joint-safety-fix/install/setup.bash"
fi
set -u
export ROS_DOMAIN_ID=0
exec python3 - "$task_root" "${PEN_HTTP_PORT:-8095}" <<'PY'
import os
from pathlib import Path
import sys
import yaml
root = Path(sys.argv[1])
profile = root / 'src/pen_description/config/tylenol.yaml'
config = yaml.safe_load(profile.read_text())
args = [sys.executable, '-m', 'pen_bringup.web_bridge', '--root', str(root),
        '--mode', 'real', '--profile', str(profile), '--port', sys.argv[2]]
if config.get('prepared_trajectory'):
    args += ['--prepared-job', str(root / config['prepared_trajectory'])]
os.execv(sys.executable, args)
PY
