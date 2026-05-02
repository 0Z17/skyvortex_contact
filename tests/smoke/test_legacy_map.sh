#!/usr/bin/env bash
set -euo pipefail

map_file="catkin_ws/src/uav_contact_core/legacy/migration_map.md"
legacy_dir="catkin_ws/src/uav_contact_core/legacy/reference_snapshot"
package_root="catkin_ws/src/uav_contact_core"

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
  dest_part_trimmed="$(printf '%s' "$dest_part" | xargs)"

  is_planned=false
  if [[ "$dest_part_trimmed" == *" (planned)" ]]; then
    is_planned=true
    dest="${dest_part_trimmed% (planned)}"
  else
    dest="$dest_part_trimmed"
  fi

  # Destination paths in migration_map.md are package-relative.
  # Only enforce existence for non-planned entries.
  if [ "$is_planned" = true ]; then
    continue
  fi

  resolved_dest="$package_root/$dest"
  [ -f "$resolved_dest" ] || {
    echo "Destination missing and not marked (planned): $resolved_dest"
    exit 1
  }
done < "$map_file"
