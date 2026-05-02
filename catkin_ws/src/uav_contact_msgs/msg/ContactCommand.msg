# Commanded interaction targets for the contact controller.

std_msgs/Header header
uav_contact_msgs/TaskPhase target_phase
geometry_msgs/PoseStamped target_pose
geometry_msgs/Twist target_twist
geometry_msgs/Wrench target_wrench
float32 max_contact_force
bool hold_position
