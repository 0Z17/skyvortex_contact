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
