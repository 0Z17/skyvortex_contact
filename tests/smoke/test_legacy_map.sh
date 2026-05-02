#!/usr/bin/env bash
set -euo pipefail

map_file="catkin_ws/src/uav_contact_core/legacy/migration_map.md"
legacy_dir="catkin_ws/src/uav_contact_core/legacy/reference_snapshot"

# migration_map.md must exist and be non-empty
[ -s "$map_file" ]

# Validate each mapping line:
# - each legacy source exists in reference_snapshot
# - destination must exist, or mapping must be marked "(planned)"
while IFS= read -r line; do
  [[ "$line" =~ ^-\  ]] || continue

  src_part="${line#- }"
  src_part="${src_part%% -> *}"

  # Normalize combined extension shorthand used in map (e.g., controller.cpp/.hpp)
  src_part="${src_part//.cpp\/.hpp/.cpp + controller.hpp}"

  # Split multiple sources on '+' and trim whitespace around each token
  while IFS= read -r src_token; do
    src="$(printf '%s' "$src_token" | xargs)"
    [ -n "$src" ] || continue
    [ -f "$legacy_dir/$src" ] || {
      echo "Missing legacy source listed in map: $legacy_dir/$src"
      exit 1
    }
  done < <(printf '%s\n' "$src_part" | tr '+' '\n')

  dest_part="${line#* -> }"
  dest="${dest_part% (planned)}"

  if [ -f "$dest" ]; then
    continue
  fi

  [[ "$line" == *" (planned)" ]] || {
    echo "Destination missing and not marked (planned): $dest"
    exit 1
  }
done < "$map_file"
