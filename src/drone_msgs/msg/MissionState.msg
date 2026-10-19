std_msgs/Header header

uint8 IDLE=0
uint8 TAKEOFF=1
uint8 EXPLORE=2
uint8 INSPECT=3
uint8 RETURN=4
uint8 LAND=5
uint8 COMPLETE=6
uint8 ABORT=7

uint8 state
string state_name
string detail
uint32 unique_victims
float32 elapsed_sec
