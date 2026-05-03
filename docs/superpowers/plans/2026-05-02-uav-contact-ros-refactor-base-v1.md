# UAV Contact ROS Refactor Base V1 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS1 Noetic baseline repository with clear node boundaries and a runnable minimal control loop for UAV contact experiments.

**Architecture:** Use three packages: `uav_contact_msgs` (message contracts), `uav_contact_core` (all runtime nodes), and `uav_contact_bringup` (launch/config). Keep `uav_motion_controller` in C++ and all other nodes in Python. Keep controller logic minimal in V1: topic ingestion, simple command fusion, limiting, and MAVROS setpoint forwarding only.

**Tech Stack:** ROS1 Noetic, catkin, rospy, roscpp, mavros_msgs, geometry_msgs, std_msgs, Python3, C++14

---

## File structure and responsibilities

- `catkin_ws/src/uav_contact_msgs/msg/*.msg`: cross-node interface definitions.
- `catkin_ws/src/uav_contact_core/scripts/*`: Python runtime nodes by responsibility.
- `catkin_ws/src/uav_contact_core/src/control/uav_motion_controller_node.cpp`: minimal MAVROS setpoint node.
- `catkin_ws/src/uav_contact_core/include/uav_contact_core/control/uav_motion_controller_node.hpp`: controller class interface.
- `catkin_ws/src/uav_contact_bringup/config/*.yaml`: all runtime params and topic names.
- `catkin_ws/src/uav_contact_bringup/launch/experiment.launch`: one-command baseline bringup.
- `catkin_ws/src/uav_contact_core/legacy/reference_snapshot/*`: immutable backup of legacy references.

---

### Task 1: Workspace and package skeleton

**Files:**
- Create: `catkin_ws/src/uav_contact_msgs/package.xml`
- Create: `catkin_ws/src/uav_contact_msgs/CMakeLists.txt`
- Create: `catkin_ws/src/uav_contact_core/package.xml`
- Create: `catkin_ws/src/uav_contact_core/CMakeLists.txt`
- Create: `catkin_ws/src/uav_contact_bringup/package.xml`
- Create: `catkin_ws/src/uav_contact_bringup/CMakeLists.txt`

- [ ] **Step 1: Write failing smoke check script**

```bash
#!/usr/bin/env bash
set -e
test -d catkin_ws/src/uav_contact_msgs
test -d catkin_ws/src/uav_contact_core
test -d catkin_ws/src/uav_contact_bringup
```

- [ ] **Step 2: Run smoke check to verify it fails**

Run: `bash tests/smoke/test_workspace_layout.sh`
Expected: FAIL with missing directories.

- [ ] **Step 3: Create minimal package manifests/build files**

```xml
<!-- package.xml skeleton -->
<package format="2">
  <name>uav_contact_core</name>
  <version>0.1.0</version>
  <description>UAV contact core nodes</description>
  <maintainer email="dev@example.com">dev</maintainer>
  <license>MIT</license>
  <buildtool_depend>catkin</buildtool_depend>
</package>
```

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(uav_contact_core)
find_package(catkin REQUIRED)
catkin_package()
```

- [ ] **Step 4: Run smoke check to verify it passes**

Run: `bash tests/smoke/test_workspace_layout.sh`
Expected: PASS (no output, exit code 0).

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_msgs catkin_ws/src/uav_contact_core catkin_ws/src/uav_contact_bringup tests/smoke/test_workspace_layout.sh
git commit -m "chore: initialize catkin workspace package skeleton"
```

### Task 2: Legacy snapshot and migration map

**Files:**
- Create: `catkin_ws/src/uav_contact_core/legacy/reference_snapshot/*`
- Create: `catkin_ws/src/uav_contact_core/legacy/migration_map.md`

- [ ] **Step 1: Write failing check for migration map**

```bash
#!/usr/bin/env bash
set -e
test -f catkin_ws/src/uav_contact_core/legacy/migration_map.md
```

- [ ] **Step 2: Run check to verify it fails**

Run: `bash tests/smoke/test_legacy_map.sh`
Expected: FAIL (file not found).

- [ ] **Step 3: Copy reference files and write map**

```markdown
# Migration Map
- traj_publisher.py -> scripts/trajectory/trajectory_server_node.py
- range_pid_controller.py -> scripts/control/dist_pid_controller.py
- controller.cpp/.hpp -> src/control/uav_motion_controller_node.cpp/.hpp
- joint_mapping.py + dynamixel_control.py -> scripts/actuation/joint_servo_bridge.py
- end_effector_client.py -> scripts/kinematics/end_effector_kinematics_node.py
```

- [ ] **Step 4: Run check to verify it passes**

Run: `bash tests/smoke/test_legacy_map.sh`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/legacy tests/smoke/test_legacy_map.sh
git commit -m "chore: snapshot legacy references and add migration map"
```

### Task 3: Message contracts (`uav_contact_msgs`)

**Files:**
- Create: `catkin_ws/src/uav_contact_msgs/msg/TaskPhase.msg`
- Create: `catkin_ws/src/uav_contact_msgs/msg/ContactCommand.msg`
- Create: `catkin_ws/src/uav_contact_msgs/msg/SafetyState.msg`
- Create: `catkin_ws/src/uav_contact_msgs/msg/EndEffectorState.msg`
- Create: `catkin_ws/src/uav_contact_msgs/msg/TrajectoryPoint.msg`
- Modify: `catkin_ws/src/uav_contact_msgs/CMakeLists.txt`
- Modify: `catkin_ws/src/uav_contact_msgs/package.xml`

- [ ] **Step 1: Write failing build check**

Run: `cd catkin_ws && catkin_make --pkg uav_contact_msgs`
Expected: FAIL with missing message generation config.

- [ ] **Step 2: Add message files and generation config**

```cmake
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs)
add_message_files(FILES TaskPhase.msg ContactCommand.msg SafetyState.msg EndEffectorState.msg TrajectoryPoint.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs)
```

- [ ] **Step 3: Run build to verify it passes**

Run: `cd catkin_ws && catkin_make --pkg uav_contact_msgs`
Expected: PASS with generated message headers/python artifacts.

- [ ] **Step 4: Run message visibility check**

Run: `source catkin_ws/devel/setup.bash && rosmsg show uav_contact_msgs/TaskPhase`
Expected: Printed TaskPhase fields.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_msgs
git commit -m "feat: add uav_contact message contracts"
```

### Task 4: Task manager node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/task/task_manager_node.py`
- Create: `catkin_ws/src/uav_contact_core/tests/test_task_manager.py`
- Modify: `catkin_ws/src/uav_contact_core/CMakeLists.txt`
- Modify: `catkin_ws/src/uav_contact_core/package.xml`

- [ ] **Step 1: Write failing unit test**

```python
def test_emergency_transition_overrides_normal_flow():
    from task_manager_node import TaskManager
    tm = TaskManager(test_mode=True)
    tm.phase = tm.APPROACH
    tm.on_safety_emergency()
    assert tm.phase == tm.EMERGENCY_RETREAT
```

- [ ] **Step 2: Run test and verify it fails**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_task_manager.py -v`
Expected: FAIL (module/class missing).

- [ ] **Step 3: Implement minimal state machine node**

```python
class TaskManager:
    IDLE=0; STABILIZE=1; APPROACH=2; INITIAL_CONTACT=3; SLIDING_CONTACT=4; RETREAT=5; EMERGENCY_RETREAT=6; FINISHED=7; ERROR=8
    def on_safety_emergency(self):
        self.phase = self.EMERGENCY_RETREAT
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_task_manager.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/task/task_manager_node.py catkin_ws/src/uav_contact_core/tests/test_task_manager.py catkin_ws/src/uav_contact_core/CMakeLists.txt catkin_ws/src/uav_contact_core/package.xml
git commit -m "feat: add task manager state machine node"
```

### Task 5: Trajectory server node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/trajectory/trajectory_server_node.py`
- Create: `catkin_ws/src/uav_contact_core/data/exp_path.csv`
- Create: `catkin_ws/src/uav_contact_core/tests/test_trajectory_server.py`

- [ ] **Step 1: Write failing parsing test**

```python
def test_load_csv_returns_waypoints():
    from trajectory_server_node import load_csv
    pts = load_csv("catkin_ws/src/uav_contact_core/data/exp_path.csv")
    assert len(pts) > 0
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_trajectory_server.py -v`
Expected: FAIL (function missing).

- [ ] **Step 3: Implement minimal CSV loader and publisher skeleton**

```python
def load_csv(path):
    import csv
    with open(path) as f:
        return [row for row in csv.DictReader(f)]
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_trajectory_server.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/trajectory/trajectory_server_node.py catkin_ws/src/uav_contact_core/data/exp_path.csv catkin_ws/src/uav_contact_core/tests/test_trajectory_server.py
git commit -m "feat: add trajectory server with csv input"
```

### Task 6: Contact controller node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/control/dist_pid_controller.py`
- Create: `catkin_ws/src/uav_contact_core/tests/test_dist_pid_controller.py`

- [ ] **Step 1: Write failing behavior tests**

```python
def test_output_zero_when_phase_disabled():
    from dist_pid_controller import DistPID
    pid = DistPID(enabled_phases={3,4})
    assert pid.compute(phase=2, desired=0.03, measured=0.05) == 0.0
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_dist_pid_controller.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement minimal PID core with clamp**

```python
def compute(self, phase, desired, measured):
    if phase not in self.enabled_phases:
        return 0.0
    err = desired - measured
    out = self.kp * err
    return max(-self.max_v, min(self.max_v, out))
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_dist_pid_controller.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/control/dist_pid_controller.py catkin_ws/src/uav_contact_core/tests/test_dist_pid_controller.py
git commit -m "feat: add phase-gated distance pid controller"
```

### Task 7: UAV motion controller (C++ base-only)

**Files:**
- Create: `catkin_ws/src/uav_contact_core/include/uav_contact_core/control/uav_motion_controller_node.hpp`
- Create: `catkin_ws/src/uav_contact_core/src/control/uav_motion_controller_node.cpp`
- Create: `catkin_ws/src/uav_contact_core/tests/test_motion_controller_logic.py`
- Modify: `catkin_ws/src/uav_contact_core/CMakeLists.txt`
- Modify: `catkin_ws/src/uav_contact_core/package.xml`

- [ ] **Step 1: Write failing logic test (Python mirror of fusion math)**

```python
def test_velocity_fusion_and_clamp():
    from test_motion_controller_logic import fuse_cmd
    v = fuse_cmd(v_ref=[0.3,0,0], v_normal=0.2, n=[1,0,0], vmax=0.25)
    assert v[0] == 0.25
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_motion_controller_logic.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement C++ node with minimal logic only**

```cpp
// base logic only
Eigen::Vector3d v_cmd = v_ref_ + v_normal_cmd_ * n_;
limitNorm(v_cmd, max_velocity_);
publishPositionTarget(v_cmd);
```

- [ ] **Step 4: Build and verify compile success**

Run: `cd catkin_ws && catkin_make --pkg uav_contact_core`
Expected: PASS, binary/node target generated.

- [ ] **Step 5: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_motion_controller_logic.py -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add catkin_ws/src/uav_contact_core/include/uav_contact_core/control/uav_motion_controller_node.hpp catkin_ws/src/uav_contact_core/src/control/uav_motion_controller_node.cpp catkin_ws/src/uav_contact_core/tests/test_motion_controller_logic.py catkin_ws/src/uav_contact_core/CMakeLists.txt catkin_ws/src/uav_contact_core/package.xml
git commit -m "feat: add minimal c++ uav motion controller for mavros setpoint"
```

### Task 8: Joint servo bridge node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/actuation/joint_servo_bridge.py`
- Create: `catkin_ws/src/uav_contact_core/tests/test_joint_servo_bridge.py`

- [ ] **Step 1: Write failing clamp/map tests**

```python
def test_joint_limit_and_pwm_mapping():
    from joint_servo_bridge import map_joint_to_pwm
    pwm = map_joint_to_pwm(2.0, jmin=-1.57, jmax=1.57, pmin=500, pmax=2500)
    assert pwm == 2500
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_joint_servo_bridge.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement mapping function and node skeleton**

```python
def map_joint_to_pwm(theta, jmin, jmax, pmin, pmax):
    th = max(jmin, min(jmax, theta))
    ratio = (th - jmin) / (jmax - jmin)
    return int(pmin + ratio * (pmax - pmin))
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_joint_servo_bridge.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/actuation/joint_servo_bridge.py catkin_ws/src/uav_contact_core/tests/test_joint_servo_bridge.py
git commit -m "feat: add joint servo bridge with limit and pwm mapping"
```

### Task 9: End-effector kinematics node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/kinematics/end_effector_kinematics_node.py`
- Create: `catkin_ws/src/uav_contact_core/tests/test_end_effector_kinematics.py`

- [ ] **Step 1: Write failing output-shape test**

```python
def test_state_contains_pose_twist_and_contact_fields():
    from end_effector_kinematics_node import build_state
    st = build_state()
    assert "normal_velocity" in st
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_end_effector_kinematics.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement minimal state builder and publisher skeleton**

```python
def build_state():
    return {"normal_velocity": 0.0, "contact_error": 0.0}
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_end_effector_kinematics.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/kinematics/end_effector_kinematics_node.py catkin_ws/src/uav_contact_core/tests/test_end_effector_kinematics.py
git commit -m "feat: add end-effector kinematics baseline node"
```

### Task 10: Safety monitor node

**Files:**
- Create: `catkin_ws/src/uav_contact_core/scripts/safety/safety_monitor_node.py`
- Create: `catkin_ws/src/uav_contact_core/tests/test_safety_monitor.py`

- [ ] **Step 1: Write failing anomaly tests**

```python
def test_roll_limit_triggers_state():
    from safety_monitor_node import evaluate_safety
    state = evaluate_safety(roll_deg=20, pitch_deg=0, max_roll=12, max_pitch=12)
    assert state == "ATTITUDE_LIMIT_EXCEEDED"
```

- [ ] **Step 2: Run test and verify failure**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_safety_monitor.py -v`
Expected: FAIL.

- [ ] **Step 3: Implement minimal evaluators and publisher skeleton**

```python
def evaluate_safety(roll_deg, pitch_deg, max_roll, max_pitch):
    if abs(roll_deg) > max_roll or abs(pitch_deg) > max_pitch:
        return "ATTITUDE_LIMIT_EXCEEDED"
    return "NORMAL"
```

- [ ] **Step 4: Run test and verify pass**

Run: `pytest catkin_ws/src/uav_contact_core/tests/test_safety_monitor.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/scripts/safety/safety_monitor_node.py catkin_ws/src/uav_contact_core/tests/test_safety_monitor.py
git commit -m "feat: add safety monitor baseline node"
```

### Task 11: Bringup configs and launch

**Files:**
- Create: `catkin_ws/src/uav_contact_bringup/config/experiment.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/task_manager.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/trajectory.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/contact_controller.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/uav_motion_controller.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/kinematics.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/servo.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/safety.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/config/topics.yaml`
- Create: `catkin_ws/src/uav_contact_bringup/launch/experiment.launch`

- [ ] **Step 1: Write failing launch check**

Run: `source catkin_ws/devel/setup.bash && roslaunch uav_contact_bringup experiment.launch --screen`
Expected: FAIL due to missing files/nodes.

- [ ] **Step 2: Add YAML parameter files and launch wiring**

```xml
<launch>
  <rosparam file="$(find uav_contact_bringup)/config/task_manager.yaml" command="load"/>
  <node pkg="uav_contact_core" type="task_manager_node.py" name="task_manager" output="screen"/>
</launch>
```

- [ ] **Step 3: Run launch validation**

Run: `source catkin_ws/devel/setup.bash && roslaunch uav_contact_bringup experiment.launch`
Expected: Nodes start without parameter missing errors.

- [ ] **Step 4: Commit**

```bash
git add catkin_ws/src/uav_contact_bringup
git commit -m "feat: add bringup configs and unified experiment launch"
```

### Task 12: End-to-end verification

**Files:**
- Create: `catkin_ws/src/uav_contact_core/tests/test_e2e_checklist.md`
- Create: `catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh`

- [ ] **Step 1: Write failing e2e checklist script**

```bash
#!/usr/bin/env bash
set -e
rostopic list | grep /mavros/setpoint_raw/local
rostopic list | grep /uav_contact/safety/state
rostopic list | grep /servo/command
```

- [ ] **Step 2: Run script and verify fail before full bringup**

Run: `bash catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh`
Expected: FAIL (topics absent).

- [ ] **Step 3: Start full bringup and re-run checks**

Run: `source catkin_ws/devel/setup.bash && roslaunch uav_contact_bringup experiment.launch`
Then run: `bash catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh`
Expected: PASS with required topics present.

- [ ] **Step 4: Record final evidence**

```markdown
# E2E evidence
- trajectory -> motion -> mavros topic: PASS
- safety -> task emergency transition: PASS
- trajectory -> servo command topic: PASS
```

- [ ] **Step 5: Commit**

```bash
git add catkin_ws/src/uav_contact_core/tests/test_e2e_checklist.md catkin_ws/src/uav_contact_core/tests/scripts/run_e2e_smoke.sh
git commit -m "test: add baseline e2e verification checklist"
```

---

## Self-review of this plan

- Spec coverage: all requested Step 1-12 are mapped one-to-one into Tasks 1-12.
- Placeholder scan: no TODO/TBD placeholders remain.
- Type consistency:
  - phase flow consistently uses `uav_contact_msgs/TaskPhase`.
  - safety flow consistently uses `uav_contact_msgs/SafetyState`.
  - motion output consistently uses `/mavros/setpoint_raw/local` + `mavros_msgs/PositionTarget`.
  - controller scope is consistently minimal and excludes advanced algorithms.
