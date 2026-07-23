#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
CATKIN_WS="${SCRIPT_DIR}/catkin_ws"
LLM_DIR="${LLM_DIR:-${HOME}/llm}"
LLM_ENTRYPOINT="${LLM_ENTRYPOINT:-main.py}"

ROSLAUNCH_PID=""
NAVI_PID=""
LLM_PID=""

stop_process() {
  local pid="${1:-}"
  if [ -n "${pid}" ] && kill -0 "${pid}" 2>/dev/null; then
    kill -INT "${pid}" 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  set +e

  echo
  echo "robot_start: stopping voice, navigation, and autodrive..."
  stop_process "${LLM_PID}"
  stop_process "${NAVI_PID}"
  sleep 0.5
  stop_process "${ROSLAUNCH_PID}"

  [ -z "${LLM_PID}" ] || wait "${LLM_PID}" 2>/dev/null
  [ -z "${NAVI_PID}" ] || wait "${NAVI_PID}" 2>/dev/null
  [ -z "${ROSLAUNCH_PID}" ] || wait "${ROSLAUNCH_PID}" 2>/dev/null
  exit "${status}"
}

trap cleanup EXIT
trap 'exit 130' INT TERM

wait_for_topic() {
  local topic="$1"
  local attempts=80
  local attempt

  for ((attempt = 1; attempt <= attempts; attempt++)); do
    if rostopic list 2>/dev/null | grep -Fxq "${topic}"; then
      return 0
    fi
    if ! kill -0 "${ROSLAUNCH_PID}" 2>/dev/null; then
      echo "robot_start: autodrive exited while waiting for ${topic}." >&2
      return 1
    fi
    sleep 0.5
  done

  echo "robot_start: timed out waiting for ${topic}." >&2
  return 1
}

cd "${REPO_DIR}"
git pull --ff-only

source /opt/ros/melodic/setup.bash

cd "${CATKIN_WS}"
catkin_make
source "${CATKIN_WS}/devel/setup.bash"

REQUIRED_LLM_FILES=(
  "${LLM_ENTRYPOINT}"
  "llm_module2.py"
  "config.py"
  "realtime_stt2.py"
  "tts_module2.py"
)

for required_file in "${REQUIRED_LLM_FILES[@]}"; do
  if [ ! -f "${LLM_DIR}/${required_file}" ]; then
    echo "robot_start: required LLM file not found: ${LLM_DIR}/${required_file}" >&2
    echo "main_node.py is not used. Keep the five required Python files in LLM_DIR." >&2
    exit 1
  fi
done

if [ -x "${LLM_DIR}/venv/bin/python3" ]; then
  LLM_PYTHON="${LLM_DIR}/venv/bin/python3"
elif [ -x "${LLM_DIR}/.venv/bin/python3" ]; then
  LLM_PYTHON="${LLM_DIR}/.venv/bin/python3"
else
  LLM_PYTHON="$(command -v python3)"
fi

echo "robot_start: starting autodrive..."
roslaunch magni_nav jetson_autodrive.launch &
ROSLAUNCH_PID=$!

wait_for_topic "/move_base/status"
wait_for_topic "/scan"
wait_for_topic "/odom"

echo "robot_start: starting navi.py in voice-command mode..."
rosrun magni_nav navi.py &
NAVI_PID=$!
sleep 1.0
if ! kill -0 "${NAVI_PID}" 2>/dev/null; then
  echo "robot_start: navi.py failed to start." >&2
  exit 1
fi

echo "robot_start: starting ${LLM_DIR}/${LLM_ENTRYPOINT}..."
(
  cd "${LLM_DIR}"
  exec "${LLM_PYTHON}" "${LLM_ENTRYPOINT}"
) &
LLM_PID=$!
sleep 1.0
if ! kill -0 "${LLM_PID}" 2>/dev/null; then
  echo "robot_start: LLM process failed to start." >&2
  exit 1
fi

echo "robot_start: all processes are running."
echo "robot_start: press Ctrl+C once to stop the complete system."

while true; do
  if ! kill -0 "${ROSLAUNCH_PID}" 2>/dev/null; then
    echo "robot_start: autodrive exited." >&2
    wait "${ROSLAUNCH_PID}" || true
    exit 1
  fi
  if ! kill -0 "${NAVI_PID}" 2>/dev/null; then
    echo "robot_start: navi.py exited." >&2
    wait "${NAVI_PID}" || true
    exit 1
  fi
  if ! kill -0 "${LLM_PID}" 2>/dev/null; then
    echo "robot_start: LLM process exited." >&2
    wait "${LLM_PID}" || true
    exit 1
  fi
  sleep 1.0
done
