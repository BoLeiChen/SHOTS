# begin: Time when the measurement was started
time begin
# end: Time when the value was measured
time end

# Name or internal ID of the sensor (commonly used when a single device contains multiple sensors)
string sensor

# Source of the sensor data, e.g. wind, temperature, rain or other.
string source_type

# Unit of measure: e.g. °C, uSv/h, neutron/s etc...
string unit

# Filled when the sensor data is text
string value_text

# Filled when the sensor provides single-value measurements
float64 value_single

# Filled when sensor generates array-like data, e.g. Energy spectrum or histograms.
float64[] value_array