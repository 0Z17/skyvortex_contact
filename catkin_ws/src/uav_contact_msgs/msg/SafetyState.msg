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
