#!/usr/bin/env bash
set -euo pipefail

required_topics=(
  "/mavros/setpoint_raw/local"
  "/uav_contact/safety/state"
  "/servo/command"
)

all_topics="$(rostopic list)"
missing=0

for topic in "${required_topics[@]}"; do
  if [[ "${all_topics}" == *"${topic}"* ]]; then
    printf 'PASS topic present: %s\n' "${topic}"
  else
    printf 'FAIL topic missing: %s\n' "${topic}" >&2
    missing=1
  fi
done

if [[ ${missing} -ne 0 ]]; then
  exit 1
fi

printf 'PASS all required topics are present\n'
