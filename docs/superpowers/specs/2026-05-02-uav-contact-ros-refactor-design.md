# UAV Contact ROS1 Refactor Design (Noetic, Base Version)

## 1. Scope and fixed constraints

This design is based on the confirmed constraints:

- ROS1 version: Noetic
- Workspace style: standard `catkin_ws/src/`
- Package strategy: single core package
- Core packages:
  - `uav_contact_msgs`
  - `uav_contact_core`
  - `uav_contact_bringup`
- Language strategy:
  - `uav_motion_controller`: C++
  - other nodes: Python
- MAVROS interface in phase 1:
  - `uav_motion_controller` directly publishes `/mavros/setpoint_raw/local`

Additional hard constraint for phase 1 controller:

- `uav_motion_controller_node.cpp/.hpp` does **not** include advanced algorithms from legacy controller code:
  - no impedance control
  - no force estimator
  - no advanced observer/fusion logic
- phase 1 controller only does minimal required logic:
  - receive required topics
  - integrate basic references
  - apply limiters and safety fallback
  - output PX4-compatible `mavros_msgs/PositionTarget`

## 2. Recommended architecture

```text
Trajectory File
    ↓
Trajectory Server
    ↓
Task Manager ← Safety Monitor
    ↓
Contact Controller
    ↓
UAV Motion Controller → MAVROS
    ↓
Joint Servo Bridge → Joint Servo
    ↓
End-effector Kinematics → End-effector
```

Responsibility boundaries:

- Trajectory only handles reference generation
- Task only handles phase/state transitions
- Contact only handles normal correction command
- Motion controller only handles integration and control output
- Servo bridge only handles joint-to-servo mapping
- Kinematics only handles end-effector state estimation
- Safety only handles monitoring and safety state publication

## 3. Target repository layout

```text
catkin_ws/
└── src/
    ├── uav_contact_msgs/
    │   ├── msg/
    │   │   ├── TaskPhase.msg
    │   │   ├── ContactCommand.msg
    │   │   ├── SafetyState.msg
    │   │   ├── EndEffectorState.msg
    │   │   └── TrajectoryPoint.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── uav_contact_core/
    │   ├── scripts/
    │   │   ├── trajectory/trajectory_server_node.py
    │   │   ├── task/task_manager_node.py
    │   │   ├── control/dist_pid_controller.py
    │   │   ├── kinematics/end_effector_kinematics_node.py
    │   │   ├── actuation/joint_servo_bridge.py
    │   │   └── safety/safety_monitor_node.py
    │   ├── src/control/uav_motion_controller_node.cpp
    │   ├── include/uav_contact_core/control/uav_motion_controller_node.hpp
    │   ├── data/exp_path.csv
    │   ├── legacy/
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── uav_contact_bringup/
        ├── launch/experiment.launch
        ├── config/
        │   ├── experiment.yaml
        │   ├── task_manager.yaml
        │   ├── trajectory.yaml
        │   ├── contact_controller.yaml
        │   ├── uav_motion_controller.yaml
        │   ├── kinematics.yaml
        │   ├── servo.yaml
        │   ├── safety.yaml
        │   └── topics.yaml
        ├── CMakeLists.txt
        └── package.xml
```

## 4. Interface contract (phase 1)

### 4.1 Trajectory Server

Inputs:
- `/uav_contact/task/phase`

Outputs:
- `/uav_contact/trajectory/reference`
- `/uav_contact/joint/reference`

Rules:
- read CSV and publish references by phase
- no MAVROS publication
- no safety decisions

### 4.2 Task Manager

Inputs:
- `/uav_contact/safety/state`
- `/uav_contact/trajectory/status` (optional in base version)

Outputs:
- `/uav_contact/task/phase`

Rules:
- single source of truth for phase machine
- handles normal progression and emergency transition

### 4.3 Contact Controller

Inputs:
- `/contact/distance`
- `/contact/force` (optional)
- `/uav_contact/task/phase`

Outputs:
- `/uav_contact/contact/normal_velocity_cmd`

Rules:
- only active in configured contact phases
- output zero outside active phases

### 4.4 UAV Motion Controller (base version)

Inputs:
- `/uav_contact/trajectory/reference`
- `/uav_contact/contact/normal_velocity_cmd`
- `/uav_contact/task/phase`
- `/uav_contact/safety/state`
- `/mavros/local_position/pose`

Outputs:
- `/mavros/setpoint_raw/local`

Rules:
- basic reference integration only
- recommended merge form: `v_cmd = v_ref + v_normal_cmd * n`
- apply per-axis and total magnitude limit
- provide safe fallback output on missing inputs/safety issues
- no advanced algorithms in phase 1

### 4.5 Joint Servo Bridge

Inputs:
- `/uav_contact/joint/reference`
- `/uav_contact/task/phase`

Outputs:
- `/servo/command`
- `/uav_contact/joint/state`

Rules:
- enforce joint limits
- map joint reference to servo protocol/PWM

### 4.6 End-effector Kinematics

Inputs:
- `/mavros/local_position/pose`
- `/uav_contact/joint/state`

Outputs:
- `/uav_contact/ee/state`
- optional split outputs for tangent/normal velocity

Rules:
- estimate pose/twist and contact-related state
- no direct actuation/control output

### 4.7 Safety Monitor

Inputs:
- `/mavros/local_position/pose`
- `/mavros/imu/data`
- `/mavros/state`
- `/contact/distance`
- `/uav_contact/ee/state`
- `/uav_contact/task/phase`

Outputs:
- `/uav_contact/safety/state`

Rules:
- monitor abnormal conditions and publish safety state only
- does not mutate other node internals

## 5. Legacy-to-new mapping

- `reference_code/traj_publisher.py` -> `trajectory_server_node.py`
- `reference_code/range_pid_controller.py` -> `dist_pid_controller.py`
- `reference_code/controller.cpp` + `reference_code/controller.hpp` -> `uav_motion_controller_node.cpp/.hpp` (minimal base version only)
- `reference_code/joint_mapping.py` + `reference_code/dynamixel_control.py` -> `joint_servo_bridge.py`
- `reference_code/end_effector_client.py` -> `end_effector_kinematics_node.py`

## 6. Execution plan with acceptance criteria (Step 1-12)

### Step 1: Organize existing code
- create `legacy/reference_snapshot/`
- copy legacy references without deletion
- document mapping

Acceptance:
- legacy snapshot exists and traceability is clear

### Step 2: Create package skeleton
- create three packages and base directories
- prepare package manifests and build files

Acceptance:
- workspace recognizes all packages

### Step 3: Define messages
- implement five core msg files
- update message generation configs

Acceptance:
- messages generate successfully

### Step 4: Migrate trajectory server
- keep CSV + phase-based reference publication
- remove phase-machine and MAVROS output logic

Acceptance:
- publishes trajectory/joint references correctly

### Step 5: Implement task manager
- implement base phase machine
- support emergency transition from safety state

Acceptance:
- deterministic phase transitions with emergency override

### Step 6: Migrate contact controller
- keep minimal distance-based PID core
- phase-gated enable and output limiting

Acceptance:
- responds correctly to distance changes and phase gating

### Step 7: Migrate UAV motion controller (base version)
- minimal integration/output logic only
- publish `PositionTarget` to MAVROS

Acceptance:
- stable output rate (target 50 Hz)
- correct fallback behavior for safety/input loss
- no advanced controller modules included

### Step 8: Migrate joint servo bridge
- migrate mapping and low-level command path
- add joint and PWM constraints

Acceptance:
- correct command publication and clamping behavior

### Step 9: Migrate end-effector kinematics
- migrate state estimation logic
- publish ee state topics

Acceptance:
- stable state outputs with no uninitialized fields

### Step 10: Implement safety monitor
- detect attitude/contact/timeout/MAVROS anomalies
- publish unified safety state

Acceptance:
- each anomaly can be triggered and observed

### Step 11: Build launch and YAML
- move hardcoded parameters into YAML
- launch only loads params and starts nodes

Acceptance:
- one launch starts base chain cleanly

### Step 12: Validate end-to-end
- single-node tests
- subsystem tests
- full integration tests

Required minimum loops:
- `trajectory_server -> uav_motion_controller -> /mavros/setpoint_raw/local`
- `safety_monitor -> task_manager -> emergency_retreat phase`
- `trajectory_server -> joint_servo_bridge -> /servo/command`

Acceptance:
- all required loops pass reproducibly

## 7. Out-of-scope for phase 1

- impedance/admittance advanced variants beyond minimal PID path
- force estimation pipelines
- separate `px4_setpoint_interface_node`
- dynamic reconfigure and advanced visualization

These are deferred to later iterations after base version validation.
