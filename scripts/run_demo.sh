#!/usr/bin/env bash
set -eo pipefail
FR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$FR3_ROOT/scripts/env.sh"
exec ros2 launch fr3_vision_grasp fr3_sim.launch.py "$@"
