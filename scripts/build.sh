#!/usr/bin/env bash
set -eo pipefail
FR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
source "$FR3_ROOT/.venv-fr3/bin/activate"
cd "$FR3_ROOT/franka_ros2_ws"
# Separate outputs avoid the old workspace's absolute paths and hardware stack.
python -m colcon --log-base log_vision build \
  --build-base build_vision --install-base install_vision \
  --packages-select franka_description fr3_vision_grasp \
  --cmake-args -DBUILD_TESTING=OFF
