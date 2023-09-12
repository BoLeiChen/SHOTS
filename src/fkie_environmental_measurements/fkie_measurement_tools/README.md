# fkie_measurement_tools

## Description 

This packages offers visualization tools for working with 3D measurements collected by the [fkie_3d_measurement_server](https://gitlab.fkie.fraunhofer.de/cms-abc/3d_abc/-/tree/master/fkie_3d_measurement_server).

## Available nodes

- ```occupancy_grid_to_point_cloud```:  Converts an occupancy grid into a 3D point cloud.
- ```measurements_to_point_cloud```: Subscribes for a point cloud, and add color values on each point, that corresponds to the interpolated measurement computed by the measurement server.

## Install dependencies

The package [fkie_ddynamic_reconfigure](https://github.com/fkie/fkie_ddynamic_reconfigure) is required for compiling this package:

```
cd ros/src
git clone https://github.com/fkie/fkie_ddynamic_reconfigure
```

## Contributors

```
Francisco J. Garcia R.
francisco.garcia.rosas@fkie.fraunhofer.de
```
