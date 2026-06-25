#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${SCRIPT_DIR}/catkin_ws"

cd "${REPO_DIR}"
git pull --ff-only

source /opt/ros/melodic/setup.bash

cd "${CATKIN_WS}"
catkin_make
source "${CATKIN_WS}/devel/setup.bash"

exec roslaunch magni_nav jetson_autodrive.launch
