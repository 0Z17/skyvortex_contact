# Migration Map
- traj_publisher.py -> scripts/trajectory/trajectory_server_node.py
- range_pid_controller.py -> scripts/control/dist_pid_controller.py
- controller.cpp/.hpp -> src/control/uav_motion_controller_node.cpp/.hpp
- joint_mapping.py + dynamixel_control.py -> scripts/actuation/joint_servo_bridge.py
- end_effector_client.py -> scripts/kinematics/end_effector_kinematics_node.py
