# header
#   stamp: Time when this message was generated
#   frame_id: Ignored
Header header

# Specify if the current arrays contain all measurements or only an update
bool full_history

# Array containing located measurements
fkie_measurement_msgs/MeasurementLocated[] located_measurements

# Array containing measurements without an specific location (unknown)
fkie_measurement_msgs/Measurement[] measurements
