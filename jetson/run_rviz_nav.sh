#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${SCRIPT_DIR}/catkin_ws"
RVIZ_CONFIG="${CATKIN_WS}/src/magni_nav/rviz/magni_nav.rviz"

JETSON_IP="${JETSON_IP:-172.20.10.10}"
MSI_IP="${MSI_IP:-172.20.10.5}"

export ROS_MASTER_URI="http://${JETSON_IP}:11311"
export ROS_IP="${MSI_IP}"
unset ROS_HOSTNAME

source /opt/ros/melodic/setup.bash
if [[ -f "${CATKIN_WS}/devel/setup.bash" ]]; then
  source "${CATKIN_WS}/devel/setup.bash"
fi

if ! hostname -I | tr ' ' '\n' | grep -Fxq "${MSI_IP}"; then
  echo "rviz_nav: this laptop does not currently own ${MSI_IP}." >&2
  echo "rviz_nav: check Wi-Fi and run: hostname -I" >&2
  exit 1
fi

if ! rosnode list >/dev/null 2>&1; then
  echo "rviz_nav: cannot contact ROS master at ${ROS_MASTER_URI}." >&2
  echo "rviz_nav: start autodrive on the Jetson and check the ROS network." >&2
  exit 1
fi

ROS_NODES="$(rosnode list)"
for required_node in /amcl /map_server /move_base /odom_publisher /rplidarNode /serial_node; do
  if ! grep -Fxq "${required_node}" <<< "${ROS_NODES}"; then
    echo "rviz_nav: required node is missing: ${required_node}" >&2
    echo "rviz_nav: restart autodrive on the Jetson." >&2
    exit 1
  fi
done

ROS_TOPICS="$(rostopic list)"
for required_topic in /map /scan /odom /wheel_ticks; do
  if ! grep -Fxq "${required_topic}" <<< "${ROS_TOPICS}"; then
    echo "rviz_nav: required topic is missing: ${required_topic}" >&2
    exit 1
  fi
done

if [[ ! -f "${RVIZ_CONFIG}" ]]; then
  echo "rviz_nav: RViz config not found: ${RVIZ_CONFIG}" >&2
  exit 1
fi

echo "rviz_nav: master=${ROS_MASTER_URI} laptop=${ROS_IP}"
echo "rviz_nav: set 2D Pose Estimate and wait for LaserScan status OK before sending a goal."
exec rviz -d "${RVIZ_CONFIG}"
