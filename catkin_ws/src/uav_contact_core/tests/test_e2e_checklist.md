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
Bringup command:
```bash
source /opt/ros/noetic/setup.bash
source /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/devel/setup.bash
roslaunch uav_contact_bringup experiment.launch
```

Rerun command:
```bash
bash /home/wsl/proj/uam_exp/.worktrees/uav-contact-refactor-base-v1/catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh
```

Result:
- Exit code: `1`
- Output:
  - `PASS topic present: /mavros/setpoint_raw/local`
  - `FAIL topic missing: /uav_contact/safety/state`
  - `FAIL topic missing: /servo/command`

## Final checklist
- [x] Pre-bringup smoke check fails (expected)
- [ ] Post-bringup smoke check passes
- [x] Evidence captured for all required topics

## Blockers and closest faithful verification path
- `/uav_contact/safety/state` is not published by the current baseline. `safety_monitor_node.py` publishes on private topic `~state` (resolves to `/safety_monitor/state`) and no remap exists in `experiment.launch`.
- `/servo/command` is not published by the current baseline. The repository has joint-servo mapping logic (`joint_servo_bridge.py`) but no launched ROS publisher node wiring for `/servo/command` in `experiment.launch`.
- Closest faithful verification path implemented: use the same smoke script pre-bringup (expected full fail) and post-bringup (partial pass where `/mavros/setpoint_raw/local` appears, while missing topics are reported explicitly).
