cmake_minimum_required(VERSION 3.0.2)
project(uav_contact_core)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mavros_msgs
  roscpp
  std_msgs
  uav_contact_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mavros_msgs roscpp std_msgs uav_contact_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(uav_motion_controller_node
  src/control/uav_motion_controller_node.cpp
)

target_link_libraries(uav_motion_controller_node
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/task/task_manager_node.py
  scripts/trajectory/trajectory_server_node.py
  scripts/end_effector/end_effector_kinematics_node.py
  scripts/safety/safety_monitor_node.py
  scripts/actuation/joint_servo_bridge.py
  scripts/actuation/joint_servo_gazebo_relay.py
  scripts/actuation/dynamixel_control.py 
  scripts/utils/rc_manager_node.py
  scripts/utils/rc_to_dynamixel_speed.py
  scripts/contact/dist_pid_controller.py
  scripts/contact/experiment_data_recorder_node.py
  scripts/end_effector/end_effector_client.py
  scripts/contact/laser_distance_relay.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
