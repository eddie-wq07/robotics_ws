#!/usr/bin/env bash
# Compatibility entry point: install the active modular workspace once.
set -euo pipefail
task_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)
printf 'The 3D pen application now uses the modular workspace at %s.\n' "$task_root"
exec "$task_root/scripts/install-pen.sh" "$@"
