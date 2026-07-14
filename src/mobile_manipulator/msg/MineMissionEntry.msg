# State of one confirmed landmine in the UGV disposal mission.
uint8 PENDING=0
uint8 WAITING_FOR_MAP=1
uint8 NAVIGATING=2
uint8 AT_STANDOFF=3
uint8 WAITING_ARM=4
uint8 CLEARED=5
uint8 DEFERRED=6
uint8 UNREACHABLE=7
uint8 MANUAL_REQUIRED=8
uint8 CARRYING=9
uint8 RETURNING_HOME=10
uint8 AT_DROPOFF=11
uint8 PLACING=12

uint32 id
uint32 map_revision
uint32 frozen_map_revision
geometry_msgs/PoseStamped mine_pose
geometry_msgs/PoseStamped approach_pose
geometry_msgs/PoseStamped dropoff_pose
geometry_msgs/PoseStamped drop_pose
uint32 disposal_slot
uint8 state
uint32 navigation_attempts
uint32 grasp_attempts
string detail
time updated_at
