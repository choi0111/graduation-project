#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${HOME}/graduation-project"
CATKIN_WS="${PROJECT_DIR}/jetson/catkin_ws"
RUN_SCRIPT="${PROJECT_DIR}/jetson/run_autodrive.sh"
BASHRC="${HOME}/.bashrc"

MARKER_START="# >>> graduation-project jetson workspace >>>"
MARKER_END="# <<< graduation-project jetson workspace <<<"

if ! grep -Fq "${MARKER_START}" "${BASHRC}"; then
  {
    echo ""
    echo "${MARKER_START}"
    echo "export GRADUATION_PROJECT_DIR=\"${PROJECT_DIR}\""
    echo "export GRADUATION_PROJECT_CATKIN_WS=\"${CATKIN_WS}\""
    echo "if [ -f \"${CATKIN_WS}/devel/setup.bash\" ]; then"
    echo "  source \"${CATKIN_WS}/devel/setup.bash\""
    echo "fi"
    echo "alias autodrive='bash ${RUN_SCRIPT}'"
    echo "${MARKER_END}"
  } >> "${BASHRC}"
fi

echo "Jetson shell configured for ${CATKIN_WS}"
echo "Open a new terminal or run: source ~/.bashrc"
echo "After that, run: autodrive"
