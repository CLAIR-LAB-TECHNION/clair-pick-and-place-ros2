#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/setup_env.sh"

NUM_CUBES="${1:-2}"
exec ros2 run ros2srrc_execution hanoi_tower_demo.py --num_cubes "$NUM_CUBES"
