# Migration Map

Destinations below are planned target files for later refactor tasks. Some destination files may not exist yet; those entries are explicitly marked "(planned)".

- traj_publisher.py -> scripts/trajectory/trajectory_server_node.py (planned)
- range_pid_controller.py -> scripts/control/dist_pid_controller.py (planned)
- controller.cpp/.hpp -> src/control/uav_motion_controller_node.cpp/.hpp (planned)
- joint_mapping.py + dynamixel_control.py -> scripts/actuation/joint_servo_bridge.py (planned)
- end_effector_client.py -> scripts/kinematics/end_effector_kinematics_node.py (planned)
