# fkie_measurement_server

## Description

This package provides a measurement server node, that collects and stores measured environmental data for any number of sensors.

It also offers measurement interpolation and computation of Region-Of-Interest areas, based on the collected sensor data.

## Install dependencies

Optional if kriging is desired:

```
pip install future pykrige --user
```

## Usage example

- Launch a measurement server with simulated data:

```
roslaunch fkie_measurement_server measurement_server.launch
```

If you want to compute interpolation, just send a request message:

```
rostopic pub /commands std_msgs/String "data: 'compute_interpolation'"
```

If you want to compute ROIs (Region Of Interest):

```
rostopic pub /commands std_msgs/String "data: 'compute_boundary_polygons'"
```

## Contributors

```
Francisco J. Garcia R.
francisco.garcia.rosas@fkie.fraunhofer.de

Menaka Naazare 
menaka.naazare@fkie.fraunhofer.de
```
