# header:
#   frame_id: TF frame to which the sensor is attached
#   stamp: Time when this message was generated
Header header

# Unique ID that identifies the device 
string unique_serial_id

# Generic name assigned by the device manufacturer
string manufacturer_device_name

# classification that groups what the device is able to measure:
#   e.g. chemical (C), biological (B), radiological (R), meteorologic (M), (W) WiFi Signal etc...
string device_classification

# Array with measurement values
fkie_measurement_msgs/MeasurementValue[] values