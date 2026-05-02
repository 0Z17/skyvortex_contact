# High-level contact task phase.
# Use string constants for readable state transitions.

std_msgs/Header header
string phase
float32 progress
string[] active_constraints

string PHASE_IDLE=IDLE
string PHASE_APPROACH=APPROACH
string PHASE_ALIGN=ALIGN
string PHASE_CONTACT=CONTACT
string PHASE_TRACK=TRACK
string PHASE_RELEASE=RELEASE
string PHASE_ABORT=ABORT
