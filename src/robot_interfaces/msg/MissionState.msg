# Delivery mission FSM state (published on /mission_state)
uint8 INITIALIZING=0
uint8 IDLE=1
uint8 MISSION_RECEIVED=2
uint8 PLANNING=3
uint8 NAVIGATING=4
uint8 ARRIVED=5
uint8 VERIFY_RFID=6
uint8 OPEN_DOOR=7
uint8 WAIT_PACKAGE_REMOVAL=8
uint8 CLOSE_DOOR=9
uint8 RETURN_HOME=10
uint8 MISSION_DONE=11
uint8 ERROR=255

std_msgs/Header header
uint8 state
string state_name
string mission_id
string detail
