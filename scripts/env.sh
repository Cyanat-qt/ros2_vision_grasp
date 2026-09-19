# Source this file in each new terminal, from the copied project location.
FR3_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f "$FR3_ROOT/franka_ros2_ws/install_vision/setup.bash" ]]; then
  echo 'Run bash scripts/setup_ubuntu.sh first.' >&2
  return 1
fi
source /opt/ros/humble/setup.bash
source "$FR3_ROOT/franka_ros2_ws/install_vision/setup.bash"
source "$FR3_ROOT/.venv-fr3/bin/activate"
# Keep this simulator separate from other robots. Override before sourcing if needed.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
# CPU rendering must not consume every core needed by ROS and the physics loop.
export LP_NUM_THREADS="${LP_NUM_THREADS:-2}"
