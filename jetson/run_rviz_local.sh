#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CATKIN_WS="${SCRIPT_DIR}/catkin_ws"
RVIZ_CONFIG="${CATKIN_WS}/src/magni_nav/rviz/magni_nav.rviz"

if [ ! -f /opt/ros/melodic/setup.bash ]; then
  echo "rviz_nav: ROS Melodic setup was not found on this Jetson." >&2
  exit 1
fi

source /opt/ros/melodic/setup.bash
if [ -f "${CATKIN_WS}/devel/setup.bash" ]; then
  source "${CATKIN_WS}/devel/setup.bash"
fi

export ROS_MASTER_URI=http://localhost:11311
unset ROS_IP
unset ROS_HOSTNAME

if ! rosnode list >/dev/null 2>&1; then
  echo "rviz_nav: cannot contact the local ROS master." >&2
  echo "rviz_nav: run robot_start first, then open rviz_nav in another terminal." >&2
  exit 1
fi

if [ ! -f "${RVIZ_CONFIG}" ]; then
  echo "rviz_nav: RViz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

echo "rviz_nav: opening the local Jetson navigation view."
echo "rviz_nav: use 2D Pose Estimate before sending a destination."
exec rviz -d "${RVIZ_CONFIG}"
