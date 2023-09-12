# pose.header:
#   stamp: Time when this message was generated
#   frame_id: global frame, e.g. "world", "map", etc... 
# pose.pose:
#   Represent the utm coordinates if the [utm_zone_number] and [utm_zone_letter] are provided,
#   pose in local coordinate system otherwise.
geometry_msgs/PoseStamped pose

# (optional) utm zone e.g. "32"
int8 utm_zone_number

# (optional) utm letter e.g. "U"
string utm_zone_letter

# Measurement associated to a given location
fkie_measurement_msgs/Measurement measurement
