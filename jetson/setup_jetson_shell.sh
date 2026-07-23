#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${HOME}/graduation-project"
CATKIN_WS="${PROJECT_DIR}/jetson/catkin_ws"
RUN_SCRIPT="${PROJECT_DIR}/jetson/run_autodrive.sh"
RUN_DC_SCRIPT="${PROJECT_DIR}/jetson/run_dc_autodrive.sh"
RUN_FULL_SCRIPT="${PROJECT_DIR}/jetson/run_full_system.sh"
BASHRC="${HOME}/.bashrc"

MARKER_START="# >>> graduation-project jetson workspace >>>"
MARKER_END="# <<< graduation-project jetson workspace <<<"

TMP_BASHRC="$(mktemp)"
awk -v start="${MARKER_START}" -v end="${MARKER_END}" '
  $0 == start {skip=1; next}
  $0 == end {skip=0; next}
  skip == 1 {next}

  /^[[:space:]]*source[[:space:]]+~\/catkin_ws\/devel\/setup\.bash[[:space:]]*$/ {
    next
  }
  /^[[:space:]]*source[[:space:]]+~\/graduation-project\/jetson\/catkin_ws\/devel\/setup\.bash[[:space:]]*$/ {
    next
  }
  /^[[:space:]]*export[[:space:]]+ROS_(MASTER_URI|IP|HOSTNAME)=/ {
    print "# disabled for Jetson-local ROS: " $0
    next
  }
  /^[[:space:]]*unset[[:space:]]+ROS_(IP|HOSTNAME)[[:space:]]*$/ {
    print "# disabled for Jetson-local ROS: " $0
    next
  }

  {print}
' "${BASHRC}" > "${TMP_BASHRC}"

{
  echo ""
  echo "${MARKER_START}"
  echo "export GRADUATION_PROJECT_DIR=\"${PROJECT_DIR}\""
  echo "export GRADUATION_PROJECT_CATKIN_WS=\"${CATKIN_WS}\""
  echo "export ROS_MASTER_URI=http://localhost:11311"
  echo "unset ROS_IP"
  echo "unset ROS_HOSTNAME"
  echo "if [ -f \"${CATKIN_WS}/devel/setup.bash\" ]; then"
  echo "  source \"${CATKIN_WS}/devel/setup.bash\""
  echo "fi"
  echo "alias autodrive='bash ${RUN_SCRIPT}'"
  echo "alias dc_autodrive='bash ${RUN_DC_SCRIPT}'"
  echo "alias robot_start='bash ${RUN_FULL_SCRIPT}'"
  echo "${MARKER_END}"
} >> "${TMP_BASHRC}"

cat "${TMP_BASHRC}" > "${BASHRC}"
rm -f "${TMP_BASHRC}"

echo "Jetson shell configured for ${CATKIN_WS}"
echo "ROS networking configured for local Jetson use (localhost)."
echo "Open a new terminal or run: source ~/.bashrc"
echo "After that, run: autodrive, dc_autodrive, or robot_start"
