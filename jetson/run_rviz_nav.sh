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

ROS_SETUP=""
if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
else
  for distro in noetic melodic; do
    if [[ -f "/opt/ros/${distro}/setup.bash" ]]; then
      ROS_SETUP="/opt/ros/${distro}/setup.bash"
      break
    fi
  done
fi

if [[ -z "${ROS_SETUP}" ]]; then
  echo "rviz_nav: no supported ROS installation found under /opt/ros." >&2
  echo "rviz_nav: run: ls /opt/ros" >&2
  exit 1
fi

# RViz only consumes standard ROS messages, so the MSI must not source a
# Jetson-built catkin devel space from a different ROS distribution.
source "${ROS_SETUP}"

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

echo "rviz_nav: ros=${ROS_DISTRO:-unknown} master=${ROS_MASTER_URI} laptop=${ROS_IP}"
echo "rviz_nav: set 2D Pose Estimate and wait for LaserScan status OK before sending a goal."
exec rviz -d "${RVIZ_CONFIG}"
