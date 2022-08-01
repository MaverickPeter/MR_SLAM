# Vector type definition (based on Kindr definitions)
uint8 TYPE_TYPELESS=0
uint8 TYPE_JERK=6
uint8 TYPE_ACCELERATION=7
uint8 TYPE_VELOCITY=8
uint8 TYPE_POSITION=9
uint8 TYPE_FORCE=10
uint8 TYPE_MOMEMTUM=11
uint8 TYPE_ANGULAR_JERK=12
uint8 TYPE_ANGULAR_ACCELERATION=13
uint8 TYPE_ANGULAR_VELOCITY=14
uint8 TYPE_TORQUE=16
uint8 TYPE_ANGULAR_MOMEMTUM=17

Header header
uint8 type
string name
geometry_msgs/Vector3 vector # Frame defined in header
geometry_msgs/Point position # Point of origin of the vector
string position_frame_id # If empty same as vector frame
