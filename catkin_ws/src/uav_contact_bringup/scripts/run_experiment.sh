#!/usr/bin/env bash
set -euo pipefail

NAMESPACE="${1:-uav_contact}"
USE_SIM_TIME="${2:-false}"
START_MAVROS="${3:-false}"

OUTPUT_DIR="${HOME}/rosbag/uav_contact"
FILE_PREFIX="uav_contact_exp"

RECORD_TOPICS=(
  "/uav_contact/trajectory/reference"
  "/mavros/local_position/pose"
  "/mavros/local_position/velocity_local"
  "/uav_contact/diagnostics/position_error_ref_minus_meas"
  "/uav_contact/diagnostics/velocity_error_ref_minus_meas"
  "/mavros/state"
  "/uav_contact/task/phase"
  "/uav_contact/safety/state"
  "/uav_contact/contact/normal_velocity_cmd"
  "/mavros/setpoint_raw/local"
  "/uav_contact/task/sliding_done"
  "/servo/command"
  "/contact/distance"
  "/joint_states"
  "/mavros/setpoint_raw/target_local"
  "/mavros/local_position/odom"
  "/mavros/imu/data"
  "/end_effector_velocity"
)

cleanup() {
  printf '\nShutting down...\n'
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "${BAG_PID}" 2>/dev/null; then
    kill -INT "${BAG_PID}" 2>/dev/null || true
    wait "${BAG_PID}" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "${LAUNCH_PID}" 2>/dev/null; then
    kill -INT "${LAUNCH_PID}" 2>/dev/null || true
    wait "${LAUNCH_PID}" 2>/dev/null || true
  fi
  printf 'Done.\n'
}

trap cleanup EXIT INT TERM

mkdir -p "${OUTPUT_DIR}"
TIMESTAMP="$(date +%Y%m%d_%H%M%S)"
BAG_PATH="${OUTPUT_DIR}/${FILE_PREFIX}_${TIMESTAMP}.bag"

printf 'Launching experiment (namespace=%s sim_time=%s mavros=%s)...\n' \
  "${NAMESPACE}" "${USE_SIM_TIME}" "${START_MAVROS}"

roslaunch uav_contact_bringup experiment.launch \
  namespace:="${NAMESPACE}" \
  use_sim_time:="${USE_SIM_TIME}" \
  start_mavros:="${START_MAVROS}" &
LAUNCH_PID=$!

printf 'Waiting for topics to become available...\n'
for topic in "${RECORD_TOPICS[@]}"; do
  while ! rostopic list 2>/dev/null | grep -qF "${topic}"; do
    sleep 0.5
  done
  printf '  OK %s\n' "${topic}"
done

printf 'Starting rosbag record: %s\n' "${BAG_PATH}"
rosbag record -O "${BAG_PATH}" "${RECORD_TOPICS[@]}" &
BAG_PID=$!

printf 'Experiment running. Press Ctrl+C to stop.\n'
wait "${LAUNCH_PID}"
