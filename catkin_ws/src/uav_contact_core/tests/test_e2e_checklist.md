# Task 12 E2E verification checklist

## Required topics
- `/mavros/setpoint_raw/local`
- `/uav_contact/safety/state`
- `/servo/command`

## Step 1: Pre-bringup failing check evidence
Command:
```bash
source /opt/ros/noetic/setup.bash
source /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/devel/setup.bash
bash /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh
```

Result:
- Exit code: `1` (expected fail)
- Output:
  - `FAIL topic missing: /mavros/setpoint_raw/local`
  - `FAIL topic missing: /uav_contact/safety/state`
  - `FAIL topic missing: /servo/command`

## Step 2: Full bringup and rerun evidence
Bringup + smoke command (timed launch):
```bash
source /opt/ros/noetic/setup.bash
source /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/devel/setup.bash
timeout 90s bash -lc 'roslaunch uav_contact_bringup experiment.launch >/tmp/uam_task12_launch.log 2>&1 & LPID=$!; sleep 12; bash /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh; RC=$?; kill $LPID >/dev/null 2>&1 || true; wait $LPID >/dev/null 2>&1 || true; exit $RC'
```

Result:
- Exit code: `0`
- Output:
  - `PASS topic present: /mavros/setpoint_raw/local`
  - `PASS topic present: /uav_contact/safety/state`
  - `PASS topic present: /servo/command`
  - `PASS all required topics are present`

## Final checklist
- [x] Pre-bringup smoke check fails (expected)
- [x] Post-bringup smoke check passes
- [x] Evidence captured for all required topics

## Blockers resolution summary
- `/uav_contact/safety/state` now exists at runtime via remap in `experiment.launch` from `safety_monitor` private `~state` publisher.
- `/servo/command` now exists at runtime via launched `joint_servo_bridge.py` publisher scaffold.
