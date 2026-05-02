cmake_minimum_required(VERSION 3.0.2)
project(uav_contact_core)

find_package(catkin REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/task/task_manager_node.py
  scripts/trajectory/trajectory_server_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
