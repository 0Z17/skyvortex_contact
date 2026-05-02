#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

[ -d "$ROOT_DIR/catkin_ws/src/uav_contact_msgs" ]
[ -d "$ROOT_DIR/catkin_ws/src/uav_contact_core" ]
[ -d "$ROOT_DIR/catkin_ws/src/uav_contact_bringup" ]
