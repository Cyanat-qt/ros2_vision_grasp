#!/usr/bin/env bash
# Run from anywhere: bash scripts/setup_ubuntu.sh
set -eo pipefail
FR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'Please install ROS2 Humble on Ubuntu 22.04 first (/opt/ros/humble).' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip python3-colcon-common-extensions \
  python3-numpy python3-opencv python3-yaml python3-pytest \
  libgl1-mesa-dri libegl1 libglfw3 \
  ros-humble-moveit ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-rviz2 ros-humble-cv-bridge ros-humble-message-filters \
  ros-humble-tf2-geometry-msgs ros-humble-vision-msgs ros-humble-control-msgs \
  ros-humble-rosgraph-msgs ros-humble-rqt-image-view
if [[ ! -x "$FR3_ROOT/.venv-fr3/bin/python" ]]; then
  /usr/bin/python3 -m venv --system-site-packages "$FR3_ROOT/.venv-fr3"
fi
source "$FR3_ROOT/.venv-fr3/bin/activate"
python -m pip install 'numpy<2' 'mujoco==3.13.0'
bash "$FR3_ROOT/scripts/build.sh"
echo 'Ready. Run: bash scripts/run_demo.sh'
