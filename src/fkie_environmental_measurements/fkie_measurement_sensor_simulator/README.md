# fkie_measurement_sensor_simulator

## Description

This package allows to simulate environmental sources such us hot-spots or radioactive elements. It assumes the measurements are continuously distributed across the space using a (configurable) propagation function.

## Demo:

Launch a basic simulation setup with an static sensor:

```
roslaunch fkie_measurement_sensor_simulator sensor_simulator.launch
```

## Usage

- Sensor simulator node:

``` xml
 <node name="SensorSimulatorNode" pkg="fkie_measurement_sensor_simulator" type="SensorSimulatorNode" clear_params="true">
  <param name="capability_group" value="sensor_simulator"/>

  <rosparam command="load" file="$(find fkie_measurement_sensor_simulator)/config/sensor_simulator_config.yaml" />

  <param name="global_frame" value="$(arg global_frame)"/>
  <param name="sensor_frame" value="$(arg sensor_frame)"/>
  <param name="rate" value="2.0"/>
  <param name="marker_size" value="0.4"/>
  <param name="topic_measurement" value="measurement"/>

  <param name="unique_serial_id" value="ASD-123"/>
  <param name="manufacturer_device_name" value="Dummy Thermometer Device"/>
  <param name="device_classification" value="M"/>

  <param name="sensor_name" value="sensor_1"/>
  <param name="sensor_source_type" value="temperature"/>
  <param name="sensor_unit" value="C"/>
</node>
```

- Example Config file ```simulator_config.yaml```:

``` yaml
sources:
  source_0:
    name: "hot-spot 1"
    intensity: 10.0
    x: 1.0
    y: 1.0
    z: 1.0
    function: "linear"
    linear_alpha: 5
    color_rgba: [0.0, 0.7, 0.0, 0.8] 
    text_color_rgba: [0.8, 0.8, 0.8, 0.8]

  source_1:
    name: "hot-spot 2"
    intensity: 10.0
    x: 1.0
    y: 3.0
    z: 1.0
    function: "inverse_squared"
    linear_alpha: 5
    color_rgba: [0.0, 0.0, 0.7, 0.8]
    text_color_rgba: [0.8, 0.8, 0.8, 0.8]

  source_2:
    name: "hot-spot 3"
    intensity: 10.0
    x: 2.0
    y: 2.0
    z: 1.0
    function: "exponential"
    exponential_decay_rate: 3
    color_rgba: [0.7, 0.0, 0.0, 0.8]
    text_color_rgba: [0.8, 0.8, 0.8, 0.8]
```

## Contributors

```
Francisco J. Garcia R.
francisco.garcia.rosas@fkie.fraunhofer.de
```