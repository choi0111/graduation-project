#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="${HOME}/graduation-project"
RUN_SCRIPT="${PROJECT_DIR}/jetson/run_rviz_nav.sh"
BASHRC="${HOME}/.bashrc"

MARKER_START="# >>> graduation-project msi rviz >>>"
MARKER_END="# <<< graduation-project msi rviz <<<"

TMP_BASHRC="$(mktemp)"
if grep -Fq "${MARKER_START}" "${BASHRC}"; then
  awk -v start="${MARKER_START}" -v end="${MARKER_END}" '
    $0 == start {skip=1; next}
    $0 == end {skip=0; next}
    skip != 1 {print}
  ' "${BASHRC}" > "${TMP_BASHRC}"
else
  cp "${BASHRC}" "${TMP_BASHRC}"
fi

{
  echo ""
  echo "${MARKER_START}"
  echo "export ROS_MASTER_URI=http://172.20.10.10:11311"
  echo "export ROS_IP=172.20.10.5"
  echo "unset ROS_HOSTNAME"
  echo "alias rviz_nav='bash ${RUN_SCRIPT}'"
  echo "${MARKER_END}"
} >> "${TMP_BASHRC}"

cat "${TMP_BASHRC}" > "${BASHRC}"
rm -f "${TMP_BASHRC}"

echo "MSI RViz shell configured."
echo "Open a new terminal or run: source ~/.bashrc"
echo "After autodrive is running on the Jetson, run: rviz_nav"
