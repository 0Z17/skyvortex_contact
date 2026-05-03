# ROS1 UAV Contact Experiment Code Refactoring Specification

> 本文档用于指导后续 coding agent 在参考已有 ROS1 实验代码的基础上，完成无人机接触实验系统的工程化重构。  
> 重构目标不是重新设计算法，而是在尽量保留已有节点功能和实验逻辑的基础上，重新划分模块职责、统一话题接口、规范参数配置、整理 launch 文件与代码结构，使系统更便于调试、复现实验和后续扩展。

---

## 1. 重构目标

当前已有代码来源于无人机接触实验的临时工程，功能可以运行，但存在如下典型问题：

1. 节点职责混杂，例如轨迹发布、阶段切换、控制模式切换、安全判断可能写在同一个脚本中。
2. 话题命名不统一，缺少统一命名空间。
3. 参数大量写死在代码中，不利于实验调参和复现。
4. MAVROS / PX4 接口逻辑与控制算法耦合较深。
5. 末端执行器、舵机、接触控制和安全监控之间的数据流不够清晰。
6. 代码缺少统一 package 结构、launch 管理和配置文件管理。

本次重构希望形成一个结构清晰的 ROS1 工程，整体架构如下：

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

其中，`Trajectory Server` 负责轨迹与关节参考，`Task Manager` 负责实验阶段切换，`Contact Controller` 负责接触距离/法向修正，`UAV Motion Controller` 负责无人机运动控制输出，`Joint Servo Bridge` 负责关节舵机接口，`End-effector Kinematics` 负责末端运动学和状态估计，`Safety Monitor` 负责异常状态检测与紧急撤离触发。

---

## 2. 总体架构

根据最终架构图，系统分为四个逻辑区域：

### 2.1 任务与轨迹层
包含以下节点：

```text
Trajectory File: exp_path.csv
Trajectory Server: trajectory_server_node.py
Task Manager: task_manager_node.py
```

职责：

- 从 `exp_path.csv` 读取实验轨迹；
- 发布无人机轨迹参考；
- 发布关节参考；
- 管理实验阶段；
- 根据安全监控结果切换任务状态；
- 将当前阶段信息提供给接触控制器、运动控制器、舵机桥接节点等。

主要参考已有代码文件：

`\reference_code\traj_publisher.py`

### 2.2 接触与安全层

包含以下节点：

```text
Contact Controller: dist_pid_controller.py
Safety Monitor: safety_monitor_node.py
```

职责：

- 接触控制器根据距离误差或接触误差输出法向修正量；
- 安全监控器根据姿态、距离、通信、末端状态等判断是否进入异常状态；
- 安全监控器向任务管理器反馈安全状态；
- 必要时触发撤离或紧急停止逻辑。

主要参考已有代码文件：

`\reference_code\range_pid_controller.py`

### 2.3 控制与执行接口层

包含以下节点：

```text
UAV Motion Controller: uav_motion_controller_node.py / .cpp
Joint Servo Bridge: joint_servo_bridge.py
End-effector Kinematics: end_effector_kinematics_node.py
```

职责：

- UAV Motion Controller 接收轨迹参考、任务阶段和接触修正，输出无人机控制指令；
- Joint Servo Bridge 接收关节参考并转换为舵机指令；
- End-effector Kinematics 根据无人机状态和关节状态计算末端位姿、速度和接触相关状态。

主要参考已有代码文件：

`\reference_code\controller.cpp`
`\reference_code\controller.hpp`
`\reference_code\joint_mapping.py`
`\reference_code\end_effector_client.py`

### 2.4 硬件与底层接口层

包含：

```text
MAVROS
Joint Servo
End-effector
```

职责：

- MAVROS 负责与 PX4 / 飞控通信；
- Joint Servo 负责执行关节动作；
- End-effector 代表实际接触机构或末端作业模块。

---

## 3. 推荐 ROS Package 结构

```text
catkin_ws/
└── src/
    ├── uav_contact_msgs/
    │   ├── msg/
    │   │   ├── TaskPhase.msg
    │   │   ├── ContactCommand.msg
    │   │   ├── SafetyState.msg
    │   │   ├── TrajectoryPoint.msg
    │   │   └── EndEffectorState.msg
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    ├── uav_contact_core/
    │   ├── scripts/
    │   │   ├── trajectory/
    │   │   │   └── trajectory_server_node.py
    │   │   ├── task/
    │   │   │   └── task_manager_node.py
    │   │   ├── control/
    │   │   │   ├── dist_pid_controller.py
    │   │   │   └── uav_motion_controller_node.py
    │   │   ├── kinematics/
    │   │   │   └── end_effector_kinematics_node.py
    │   │   ├── actuation/
    │   │   │   └── joint_servo_bridge.py
    │   │   └── safety/
    │   │       └── safety_monitor_node.py
    │   │
    │   ├── src/
    │   │   ├── control/
    │   │   │   └── uav_motion_controller_node.cpp
    │   │   └── common/
    │   │       └── utils.cpp
    │   │
    │   ├── include/
    │   │   └── uav_contact_core/
    │   │       ├── control/
    │   │       └── common/
    │   │
    │   ├── data/
    │   │   └── exp_path.csv
    │   │
    │   ├── legacy/
    │   │   └── README.md
    │   │
    │   ├── CMakeLists.txt
    │   └── package.xml
    │
    └── uav_contact_bringup/
        ├── launch/
        │   ├── experiment.launch
        │   ├── simulation.launch
        │   ├── real_robot.launch
        │   └── record_bag.launch
        ├── config/
        │   ├── experiment.yaml
        │   ├── task_manager.yaml
        │   ├── trajectory.yaml
        │   ├── contact_controller.yaml
        │   ├── uav_motion_controller.yaml
        │   ├── kinematics.yaml
        │   ├── servo.yaml
        │   ├── safety.yaml
        │   ├── topics.yaml
        │   └── record.yaml
        ├── rviz/
        ├── bag/
        ├── CMakeLists.txt
        └── package.xml
```

---

## 4. 节点职责定义

### 4.1 Trajectory Server

文件建议：

```text
trajectory_server_node.py
```

输入：

```text
exp_path.csv
/task/phase
```

输出：

```text
/uav_contact/trajectory/reference
/uav_contact/joint/reference
```

主要职责：

1. 读取 `exp_path.csv`。
2. 根据当前任务阶段发布对应的轨迹参考。
3. 发布无人机参考位置、速度或加速度。
4. 发布关节参考角度。
5. 不负责安全判断。
6. 不直接发布 MAVROS setpoint。
7. 不负责 offboard 切换、arming 或 type_mask 处理。

建议逻辑：

- 在 `STABILIZE` 阶段，可发布悬停参考或不发布轨迹。
- 在 `APPROACH` 阶段，发布接近轨迹。
- 在 `INITIAL_CONTACT` 阶段，发布低速接近或接触初始化参考。
- 在 `SLIDING_CONTACT` 阶段，发布沿表面的滑动轨迹。
- 在 `RETREAT` 阶段，发布撤离轨迹或停止轨迹输出。
- 在 `EMERGENCY_RETREAT` 阶段，不再按原始轨迹推进。

---

### 4.2 Task Manager

文件建议：

```text
task_manager_node.py
```

输入：

```text
/uav_contact/safety/state
/uav_contact/trajectory/status
可选：/mavros/state
可选：用户启动命令
```

输出：

```text
/uav_contact/task/phase
```

主要职责：

1. 维护实验阶段状态机。
2. 根据时间、轨迹进度和安全状态进行阶段切换。
3. 作为所有节点的统一任务状态来源。
4. 不直接计算控制指令。
5. 不直接发送 MAVROS setpoint。
6. 不直接发送舵机底层指令。

建议任务阶段：

```text
IDLE
STABILIZE
APPROACH
INITIAL_CONTACT
SLIDING_CONTACT
RETREAT
EMERGENCY_RETREAT
FINISHED
ERROR
```

推荐状态切换：

```text
IDLE
  → STABILIZE
  → APPROACH
  → INITIAL_CONTACT
  → SLIDING_CONTACT
  → RETREAT
  → FINISHED
```

异常切换：

```text
任意阶段 → EMERGENCY_RETREAT → ERROR 或 FINISHED
```

建议参数：

```yaml
task_manager:
  stabilize_duration: 5.0
  approach_duration: 30.0
  initial_contact_duration: 5.0
  retreat_duration: 20.0
  auto_start: false
```

---

### 4.3 Contact Controller

文件建议：

```text
dist_pid_controller.py
```

输入：

```text
/contact/distance
/contact/force
/uav_contact/task/phase
可选：/uav_contact/ee/state
```

输出：

```text
/uav_contact/contact/normal_velocity_cmd
```

主要职责：

1. 根据接触距离误差或接触力误差计算法向修正速度。
2. 在非接触阶段自动失能或输出 0。
3. 在接触阶段启用 PID 或导纳控制。
4. 输出法向修正速度或法向位置偏置。
5. 不负责全局轨迹跟踪。
6. 不直接操作 MAVROS。

推荐控制形式：

```text
normal_velocity_cmd = PID(desired_distance - measured_distance)
```

或：

```text
normal_offset_cmd = admittance_filter(contact_force_error)
```

建议参数：

```yaml
contact_controller:
  enabled_phases:
    - INITIAL_CONTACT
    - SLIDING_CONTACT
  desired_distance: 0.03
  max_normal_velocity: 0.08
  pid:
    kp: 0.8
    ki: 0.0
    kd: 0.05
  filter:
    enable_low_pass: true
    cutoff_frequency: 5.0
```

---

### 4.4 UAV Motion Controller

文件建议：

```text
uav_motion_controller_node.py
```

或：

```text
uav_motion_controller_node.cpp
```

输入：

```text
/uav_contact/trajectory/reference
/uav_contact/contact/normal_velocity_cmd
/uav_contact/task/phase
/uav_contact/safety/state
/mavros/local_position/pose
```

输出：

```text
/mavros/setpoint_raw/local
```

或先输出内部话题：

```text
/uav_contact/control/setpoint_raw
```

主要职责：

1. 执行无人机主运动控制。
2. 融合轨迹参考和法向接触修正。
3. 根据任务阶段选择控制模式。
4. 生成 MAVROS `mavros_msgs/PositionTarget` 控制指令。
5. 负责速度、加速度、位置参考的限幅。
6. 在紧急状态下输出撤离、悬停或安全速度。
7. 尽量避免把任务状态机写在本节点内部，只响应 `task_phase`。

建议融合逻辑：

```text
v_cmd = v_ref + v_normal_cmd * n
```

其中：

- `v_ref` 来自轨迹参考；
- `v_normal_cmd` 来自 Contact Controller；
- `n` 是表面法向或无人机/末端法向方向；
- 最终输出限幅后的速度控制指令。

建议控制模式：

```text
STABILIZE:         速度为 0 或位置保持
APPROACH:          轨迹跟踪，接触修正关闭
INITIAL_CONTACT:   低速接近，接触修正开启
SLIDING_CONTACT:   切向轨迹 + 法向修正
RETREAT:           撤离速度或撤离轨迹
EMERGENCY_RETREAT: 紧急撤离或悬停
```

建议参数：

```yaml
uav_motion_controller:
  control_rate: 50.0
  output_mode: velocity
  max_velocity: 0.25
  max_acceleration: 0.5
  max_normal_velocity: 0.08
  max_tangent_velocity: 0.15
  use_mavros_setpoint_raw: true
  frame_id: map
```

注意：

- 如果直接发布 `/mavros/setpoint_raw/local`，必须保证 setpoint 发布频率稳定，通常建议大于 20 Hz。
- 如果后续希望更工程化，可以单独拆出 `px4_setpoint_interface_node` 管理 OFFBOARD、arming、type_mask 等逻辑。

---

### 4.5 Joint Servo Bridge

文件建议：

```text
joint_servo_bridge.py
```

输入：

```text
/uav_contact/joint/reference
/uav_contact/task/phase
```

输出：

```text
/servo/command
/uav_contact/joint/state
```

主要职责：

1. 接收关节参考角度。
2. 做关节限幅。
3. 做角度到 PWM、串口指令或底层舵机协议的转换。
4. 发布关节状态。
5. 不负责轨迹规划。
6. 不负责 UAV 主控制。
7. 不负责实验阶段切换，只根据阶段决定是否允许执行。

建议参数：

```yaml
joint_servo_bridge:
  servo_rate: 50.0
  joint_limits:
    joint_1:
      min: -1.57
      max: 1.57
  pwm_limits:
    min: 500
    max: 2500
  neutral_pwm: 1500
  enable_in_phases:
    - APPROACH
    - INITIAL_CONTACT
    - SLIDING_CONTACT
    - RETREAT
```

备注：

具体joint server接口要与`\reference_code\joint_mapping.py` 和  `\reference_code\dynamixel_control.py`中的接口对应上

---

### 4.6 End-effector Kinematics

文件建议：

```text
end_effector_kinematics_node.py
```

输入：

```text
/mavros/local_position/pose
/uav_contact/joint/state
可选：/tf
可选：/robot_description
```

输出：

```text
/uav_contact/ee/state
/uav_contact/ee/tangent_velocity
/uav_contact/ee/normal_velocity
```

主要职责：

1. 根据无人机位姿和关节状态计算末端位姿。
2. 根据差分或雅可比计算末端速度。
3. 将末端速度分解到切向和法向方向。
4. 发布末端状态供控制器和安全监控器使用。
5. 不直接控制无人机。
6. 不直接控制舵机。

推荐输出内容：

```text
end-effector position
end-effector orientation
end-effector velocity
tangent velocity
normal velocity
contact error
surface normal
```

建议参数：

```yaml
end_effector_kinematics:
  rate: 50.0
  base_frame: map
  uav_frame: base_link
  ee_frame: end_effector
  use_tf: true
  velocity_filter:
    enable: true
    cutoff_frequency: 5.0
```

备注：

具体end_effector_kinematics_node接口要与`\reference_code\end_effector_client.py` 中的接口对应上

---

### 4.7 Safety Monitor

文件建议：

```text
safety_monitor_node.py
```

输入：

```text
/mavros/local_position/pose
/mavros/imu/data
/mavros/state
/contact/distance
/uav_contact/ee/state
/uav_contact/task/phase
```

输出：

```text
/uav_contact/safety/state
```

可选输出：

```text
/uav_contact/safety/emergency_cmd
```

主要职责：

1. 检测接触距离快速增大。
2. 检测接触脱离。
3. 检测俯仰角或横滚角过大。
4. 检测通信超时。
5. 检测末端状态异常。
6. 检测 MAVROS / PX4 状态异常。
7. 向 Task Manager 反馈安全状态。
8. 必要时触发 `EMERGENCY_RETREAT`。

建议安全状态：

```text
NORMAL
CONTACT_LOSS
DISTANCE_JUMP
ATTITUDE_LIMIT_EXCEEDED
MAVROS_DISCONNECTED
SENSOR_TIMEOUT
EMERGENCY_RETREAT_REQUIRED
ERROR
```

备注：

具体逻辑参考`reference_code\range_pid_controller.py` 中的安全策略

建议参数：

```yaml
safety_monitor:
  rate: 50.0
  max_roll_deg: 12.0
  max_pitch_deg: 12.0
  contact_loss_distance: 0.08
  distance_jump_threshold: 0.03
  sensor_timeout: 0.5
  mavros_timeout: 0.5
```

---

## 5. 推荐话题接口

建议所有自定义话题统一放在 `/uav_contact` 命名空间下。

### 5.1 任务与轨迹

```text
/uav_contact/task/phase
/uav_contact/trajectory/reference
/uav_contact/trajectory/status
/uav_contact/joint/reference
```

### 5.2 接触控制

```text
/contact/distance
/contact/force
/uav_contact/contact/normal_velocity_cmd
/uav_contact/contact/normal_offset_cmd
```

### 5.3 末端状态

```text
/uav_contact/ee/state
/uav_contact/ee/tangent_velocity
/uav_contact/ee/normal_velocity
```

### 5.4 控制输出

```text
/uav_contact/control/setpoint_raw
/mavros/setpoint_raw/local
```

如果暂时不拆 PX4 接口节点，`UAV Motion Controller` 可以直接发布：

```text
/mavros/setpoint_raw/local
```

### 5.5 舵机接口

```text
/uav_contact/joint/reference
/uav_contact/joint/state
/servo/command
```

### 5.6 安全状态

```text
/uav_contact/safety/state
/uav_contact/safety/emergency_cmd
```

### 5.7 MAVROS 反馈

```text
/mavros/local_position/pose
/mavros/imu/data
/mavros/state
```

---

## 6. 推荐自定义消息

### 6.1 TaskPhase.msg

```text
std_msgs/Header header

uint8 IDLE=0
uint8 STABILIZE=1
uint8 APPROACH=2
uint8 INITIAL_CONTACT=3
uint8 SLIDING_CONTACT=4
uint8 RETREAT=5
uint8 EMERGENCY_RETREAT=6
uint8 FINISHED=7
uint8 ERROR=8

uint8 phase
float64 elapsed_time
bool enable_trajectory
bool enable_contact_control
bool enable_servo
bool enable_uav_control
string description
```

---

### 6.2 ContactCommand.msg

```text
std_msgs/Header header

bool enabled
geometry_msgs/Vector3 normal_direction
float64 normal_velocity
float64 normal_offset
float64 distance_error
float64 measured_distance
float64 desired_distance
```

---

### 6.3 SafetyState.msg

```text
std_msgs/Header header

uint8 NORMAL=0
uint8 CONTACT_LOSS=1
uint8 DISTANCE_JUMP=2
uint8 ATTITUDE_LIMIT_EXCEEDED=3
uint8 MAVROS_DISCONNECTED=4
uint8 SENSOR_TIMEOUT=5
uint8 EMERGENCY_RETREAT_REQUIRED=6
uint8 ERROR=7

uint8 state
bool safe
bool require_emergency_retreat
string reason
```

---

### 6.4 EndEffectorState.msg

```text
std_msgs/Header header

geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Vector3 surface_normal
geometry_msgs/Vector3 tangent_velocity
float64 normal_velocity
float64 contact_error
bool contact_valid
```

---

### 6.5 TrajectoryPoint.msg

```text
std_msgs/Header header

geometry_msgs/Pose pose_ref
geometry_msgs/Twist twist_ref
geometry_msgs/Accel accel_ref

float64[] joint_position_ref
float64 path_s
bool valid
```

---

## 7. 参数文件建议

### 7.1 experiment.yaml

```yaml
experiment:
  name: uav_contact_exp
  trajectory_file: "$(find uav_contact_trajectory)/data/exp_path.csv"
  namespace: "/uav_contact"
  use_sim_time: false
```

---

### 7.2 task_manager.yaml

```yaml
task_manager:
  rate: 20.0
  auto_start: false
  stabilize_duration: 5.0
  approach_duration: 30.0
  initial_contact_duration: 5.0
  retreat_duration: 20.0
```

---

### 7.3 contact_controller.yaml

```yaml
contact_controller:
  rate: 50.0
  desired_distance: 0.03
  max_normal_velocity: 0.08
  pid:
    kp: 0.8
    ki: 0.0
    kd: 0.05
  enabled_phases:
    - INITIAL_CONTACT
    - SLIDING_CONTACT
```

---

### 7.4 uav_motion_controller.yaml

```yaml
uav_motion_controller:
  rate: 50.0
  output_topic: "/mavros/setpoint_raw/local"
  control_mode: "velocity"
  max_velocity: 0.25
  max_normal_velocity: 0.08
  max_tangent_velocity: 0.15
  frame_id: "map"
  yaw_mode: "hold"
```

---

### 7.5 safety.yaml

```yaml
safety_monitor:
  rate: 50.0
  max_roll_deg: 12.0
  max_pitch_deg: 12.0
  contact_loss_distance: 0.08
  distance_jump_threshold: 0.03
  sensor_timeout: 0.5
  mavros_timeout: 0.5
```

---

### 7.6 servo.yaml

```yaml
joint_servo_bridge:
  rate: 50.0
  command_topic: "/servo/command"
  joint_limits:
    joint_1:
      min: -1.57
      max: 1.57
  pwm_limits:
    min: 500
    max: 2500
  neutral_pwm: 1500
```

---

## 8. Launch 文件建议

### 8.1 experiment.launch

```xml
<launch>
  <arg name="namespace" default="uav_contact"/>
  <arg name="use_sim_time" default="false"/>

  <param name="/use_sim_time" value="$(arg use_sim_time)"/>

  <rosparam file="$(find uav_contact_bringup)/config/experiment.yaml" command="load"/>
  <rosparam file="$(find uav_contact_task)/config/task_manager.yaml" command="load"/>
  <rosparam file="$(find uav_contact_trajectory)/config/trajectory.yaml" command="load"/>
  <rosparam file="$(find uav_contact_control)/config/contact_controller.yaml" command="load"/>
  <rosparam file="$(find uav_contact_control)/config/uav_motion_controller.yaml" command="load"/>
  <rosparam file="$(find uav_contact_safety)/config/safety.yaml" command="load"/>
  <rosparam file="$(find uav_contact_actuation)/config/servo.yaml" command="load"/>

  <group ns="$(arg namespace)">
    <node pkg="uav_contact_task" type="task_manager_node.py" name="task_manager" output="screen"/>
    <node pkg="uav_contact_trajectory" type="trajectory_server_node.py" name="trajectory_server" output="screen"/>
    <node pkg="uav_contact_control" type="dist_pid_controller.py" name="contact_controller" output="screen"/>
    <node pkg="uav_contact_control" type="uav_motion_controller_node.py" name="uav_motion_controller" output="screen"/>
    <node pkg="uav_contact_kinematics" type="end_effector_kinematics_node.py" name="end_effector_kinematics" output="screen"/>
    <node pkg="uav_contact_actuation" type="joint_servo_bridge.py" name="joint_servo_bridge" output="screen"/>
    <node pkg="uav_contact_safety" type="safety_monitor_node.py" name="safety_monitor" output="screen"/>
  </group>
</launch>
```

注意：

如果节点内部使用绝对话题名，例如 `/mavros/local_position/pose`，则不会受到 `<group ns="uav_contact">` 影响。  
建议自定义话题使用相对话题名，MAVROS 话题使用绝对话题名。

---

## 9. Coding Agent 重构步骤建议

后续 coding agent 应按以下顺序执行重构，避免一次性大改造成系统不可运行。

### Step 1：整理现有代码

1. 扫描当前 ROS workspace。
2. 识别已有脚本对应的功能：
   - 轨迹发布；
   - 距离 PID；
   - 主控制器；
   - 舵机映射；
   - 末端运动学；
   - 安全判断；
   - MAVROS setpoint 发布。
3. 不要立刻删除旧代码。
4. 将旧代码先移动到 `legacy/` 或保留在原目录，并记录其功能。

---

### Step 2：创建新 package 结构

1. 创建推荐 package。
2. 创建 `config/`、`launch/`、`scripts/`、`src/`、`msg/` 目录。
3. 添加 `package.xml` 和 `CMakeLists.txt`。
4. 确认 `catkin_make` 或 `catkin build` 可以通过。

---

### Step 3：定义消息接口

1. 先实现：
   - `TaskPhase.msg`
   - `ContactCommand.msg`
   - `SafetyState.msg`
   - `EndEffectorState.msg`
   - `TrajectoryPoint.msg`
2. 修改 `CMakeLists.txt` 和 `package.xml`，确保消息可以生成。
3. 使用 `rostopic echo` 测试消息发布。

---

### Step 4：迁移 Trajectory Server

1. 从旧 `traj_publisher_velocity.py` 中提取轨迹读取逻辑。
2. 移除其中的阶段切换逻辑。
3. 移除其中的 MAVROS 控制逻辑。
4. 只保留：
   - 读取 csv；
   - 根据 phase 发布 trajectory reference；
   - 发布 joint reference。

---

### Step 5：实现 Task Manager

1. 新建 `task_manager_node.py`。
2. 实现基本状态机。
3. 支持按时间自动切换阶段。
4. 支持接收 Safety Monitor 的异常状态。
5. 出现异常时切换到 `EMERGENCY_RETREAT`。

---

### Step 6：迁移 Contact Controller

1. 从旧 `range_pid_controller.py` 或类似代码中提取 PID 逻辑。
2. 统一改名为 `dist_pid_controller.py`。
3. 输入距离或力反馈。
4. 输出 `ContactCommand` 或法向速度。
5. 根据任务阶段决定是否启用。

---

### Step 7：迁移 UAV Motion Controller

1. 从旧 `control_node.cpp` 或相关控制脚本中提取控制逻辑。
2. 订阅轨迹、接触修正、任务阶段、安全状态和 MAVROS 反馈。
3. 输出 `/mavros/setpoint_raw/local` 或 `/uav_contact/control/setpoint_raw`。
4. 确保 setpoint 发布频率稳定。
5. 将 type_mask 设置集中在本节点或后续独立 PX4 接口节点中。

---

### Step 8：迁移 Joint Servo Bridge

1. 从旧 `joint_mapping.py` 中提取舵机映射逻辑。
2. 添加关节限幅。
3. 添加参数化 PWM 范围。
4. 发布关节状态。
5. 根据任务阶段决定是否允许舵机执行。

---

### Step 9：迁移 End-effector Kinematics

1. 从旧 `end_effector_client.py` 中提取末端运动学逻辑。
2. 改名为 `end_effector_kinematics_node.py`。
3. 订阅 UAV 位姿和关节状态。
4. 发布末端状态。
5. 如有切向/法向分解逻辑，应放在该节点中。

---

### Step 10：实现 Safety Monitor

1. 从旧控制器或临时代码中提取安全判断逻辑。
2. 统一放入 `safety_monitor_node.py`。
3. 检测：
   - 姿态角过大；
   - 接触距离突变；
   - 接触脱离；
   - 传感器超时；
   - MAVROS 状态异常。
4. 发布 `SafetyState`。
5. 不直接修改其他节点状态，只通过话题通知 Task Manager。

---

### Step 11：编写 Launch 和 YAML

1. 编写统一 `experiment.launch`。
2. 将所有硬编码参数迁移到 YAML。
3. 所有节点启动参数从 ROS parameter server 读取。
4. launch 中只负责加载参数和启动节点。

---

### Step 12：测试与验证

按以下顺序测试：

1. `roscore` 启动。
2. 单独启动 `trajectory_server_node.py`，确认轨迹读取正常。
3. 启动 `task_manager_node.py`，确认 phase 正常切换。
4. 启动 `contact_controller.py`，手动发布距离，确认法向速度输出。
5. 启动 `uav_motion_controller_node.py`，确认 setpoint 输出。
6. 启动 `joint_servo_bridge.py`，确认舵机命令正常。
7. 启动 `safety_monitor_node.py`，模拟异常输入，确认状态切换。
8. 最后整体 launch 测试。

---

## 10. 代码风格建议

### 10.1 Python 节点

建议统一结构：

```python
#!/usr/bin/env python3
import rospy

class NodeName:
    def __init__(self):
        self.load_params()
        self.setup_pub_sub()

    def load_params(self):
        pass

    def setup_pub_sub(self):
        pass

    def spin(self):
        rate = rospy.Rate(self.rate)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()

    def update(self):
        pass

if __name__ == "__main__":
    rospy.init_node("node_name")
    node = NodeName()
    node.spin()
```

### 10.2 C++ 节点

建议使用 class 封装：

```cpp
class UavMotionController
{
public:
  UavMotionController(ros::NodeHandle& nh, ros::NodeHandle& pnh);
  void spin();

private:
  void loadParams();
  void setupRos();
  void update(const ros::TimerEvent& event);

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
```

---

## 11. 命名规范

### 11.1 文件命名

使用小写加下划线：

```text
trajectory_server_node.py
task_manager_node.py
dist_pid_controller.py
uav_motion_controller_node.py
joint_servo_bridge.py
end_effector_kinematics_node.py
safety_monitor_node.py
```

### 11.2 ROS 节点命名

```text
trajectory_server
task_manager
contact_controller
uav_motion_controller
joint_servo_bridge
end_effector_kinematics
safety_monitor
```

### 11.3 话题命名

自定义系统内部话题统一放到：

```text
/uav_contact/...
```

MAVROS 话题保留：

```text
/mavros/...
```

### 11.4 参数命名

使用 YAML 分组：

```yaml
task_manager:
contact_controller:
uav_motion_controller:
joint_servo_bridge:
end_effector_kinematics:
safety_monitor:
```

---

## 12. 重要设计约束

后续 coding agent 必须尽量遵守以下约束：

1. 不要让 Trajectory Server 直接控制 MAVROS。
2. 不要让 Contact Controller 直接发布无人机 setpoint。
3. 不要让 Safety Monitor 直接改写控制器内部变量，而应通过话题发布安全状态。
4. 不要让 Joint Servo Bridge 参与轨迹规划。
5. 不要把实验阶段切换逻辑分散在多个节点中。
6. 所有硬编码时间、阈值、PID 参数应迁移到 YAML。
7. 每个节点应能单独启动和单独测试。
8. 每个节点应在输入缺失时进入安全默认状态。
9. 所有控制输出应有限幅。
10. 所有关键输入应有超时检测。
11. 重构过程中应保留 legacy 代码，直到新架构验证完成。
12. 不要在没有测试的情况下改变已有控制算法核心逻辑。

---

## 13. 最小可运行版本目标

第一阶段不需要一次性完成全部功能。最小可运行版本应包括：

```text
trajectory_server_node.py
task_manager_node.py
dist_pid_controller.py
uav_motion_controller_node.py
joint_servo_bridge.py
safety_monitor_node.py
experiment.launch
相关 YAML 配置
```

最小闭环：

```text
exp_path.csv
  → trajectory_server
  → uav_motion_controller
  → /mavros/setpoint_raw/local
```

同时保留：

```text
safety_monitor
  → task_manager
  → emergency_retreat phase
```

舵机链路：

```text
trajectory_server
  → joint_servo_bridge
  → /servo/command
```

---

## 14. 后续可扩展方向

在基础重构完成后，可以进一步扩展：

1. 单独增加 `px4_setpoint_interface_node`，将 OFFBOARD、arming、type_mask 从主控制器中拆出。
2. 增加 rosbag 自动记录节点或 launch。
3. 增加 RViz 可视化 marker。
4. 增加动态参数调整，例如 dynamic_reconfigure。
5. 增加仿真模式和实机模式参数切换。
6. 增加状态机可视化。
7. 增加单元测试和 roslaunch test。
8. 增加故障注入测试脚本。

---

## 15. 推荐重构原则总结

本次重构的核心原则是：

```text
轨迹只管参考；
任务只管阶段；
接触只管法向修正；
控制只管融合与输出；
舵机只管执行映射；
运动学只管状态估计；
安全只管监控与报警；
MAVROS 只作为底层飞控接口。
```

后续 coding agent 应优先保证架构清晰、接口稳定和原有实验功能可复现，再逐步优化控制算法和工程细节。
