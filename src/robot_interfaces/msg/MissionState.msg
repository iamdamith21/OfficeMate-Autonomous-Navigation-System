# Delivery mission FSM state (published on /mission_state)
#
# Two-leg delivery: base -> sender (load) -> recipient (unload) -> base.
# Numbering is contiguous and ordered along the happy path, so the webapp can
# render a progress bar straight from the value.
uint8 IDLE=0
uint8 WAIT_FOR_REQUEST=1
uint8 NAVIGATE_TO_SENDER=2
uint8 ARRIVED_AT_SENDER=3
uint8 OPEN_SENDER_DOOR=4
uint8 WAIT_FOR_FILE=5
uint8 CLOSE_SENDER_DOOR=6
uint8 NAVIGATE_TO_RECIPIENT=7
uint8 ARRIVED_AT_RECIPIENT=8
uint8 WAIT_FOR_RFID=9
uint8 VERIFY_RFID=10
uint8 OPEN_RECIPIENT_DOOR=11
uint8 WAIT_FOR_FILE_REMOVAL=12
uint8 CLOSE_RECIPIENT_DOOR=13
uint8 RETURN_TO_BASE=14
uint8 RETURN_TO_SENDER=15
uint8 MISSION_COMPLETE=16
uint8 MISSION_FAILED=17

std_msgs/Header header
uint8 state
string state_name
string mission_id
string detail

# Blackboard fields the web app needs to render mission status without having
# to infer anything from the state number alone.
uint8 retry_count
bool file_present          # latest IR reading
string door_state          # OPEN | CLOSED | MOVING | UNKNOWN
float32 seconds_in_state
