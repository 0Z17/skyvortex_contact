#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
WS_SRC="$ROOT_DIR/catkin_ws/src"

packages=(
  "uav_contact_msgs"
  "uav_contact_core"
  "uav_contact_bringup"
)

for pkg in "${packages[@]}"; do
  pkg_dir="$WS_SRC/$pkg"

  [ -d "$pkg_dir" ]
  [ -f "$pkg_dir/package.xml" ]
  [ -f "$pkg_dir/CMakeLists.txt" ]
  grep -Eq "<name>[[:space:]]*$pkg[[:space:]]*</name>" "$pkg_dir/package.xml"
done
