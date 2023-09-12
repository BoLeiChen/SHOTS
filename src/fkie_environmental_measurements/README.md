# Environmental Measurements: Storage, Simulation, Visualization and Tools

This repository contains ROS packages for storing environmental measurements using a generic message interface. Packages include simulation of sources, storage, visualization and tools. Measurement interpolation and computation of Region of Interest is also supported.

![ms_2](https://user-images.githubusercontent.com/748097/169493930-e93c04e3-8fc0-48e1-a086-8087b551316f.png)

## Package Description

- [fkie_measurement](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement) Meta-package with the dependencies
- [fkie_measurement_msgs](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement_msgs) Definition of generic measurement messages
- [fkie_measurement_server](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement_server) Collects and storages measurement data of any number of sensors
- [fkie_measurement_sensor_simulator](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement_sensor_simulator) Simulates environmental sources, such as hot-spots or air pollution.
- [fkie_measurement_tools](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement_tools) Visualization tools using point clouds and occupancy grids.

Detailed information about usage is provided on each package.

## Compilation

If you want to use all included packages, you can just compile the meta-package [fkie_measurement](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement) using [catkin tools](https://catkin-tools.readthedocs.io/en/latest/installing.html):

- Install APT dependencies:

```
sudo apt install ros-noetic-octomap-ros ros-noetic-pcl-ros ros-noetic-grid-map-core ros-noetic-grid-map-ros python3-sklearn  python3-skimage python3-scipy
```

- Install Python dependencies:

```
pip install future pykrige --user
```

- Get the code and compile:

```
cd ros/src
git clone https://github.com/fkie/fkie_environmental_measurements
cd fkie_environmental_measurements/fkie_measurement
catkin build --this
```

## Usage

A demo using the [measurement_server](https://github.com/fkie/fkie_environmental_measurements/tree/main/fkie_measurement_server) and the [measurement_sensor_simulator](https://github.com/fkie/fkie_measurement/tree/main/fkie_measurement_sensor_simulator) can be execute using:

```
cd ros
source devel/setup.bash
roslaunch fkie_measurement_server measurement_server.launch
```

If you want to compute interpolation and ROIs (Region Of Interest), just send the corresponding request message:

```
rostopic pub /commands std_msgs/String "data: 'compute_interpolation'"
```

or

```
rostopic pub /commands std_msgs/String "data: 'compute_boundary_polygons'"
```

## Publication

If you use this work for scientific purposes, please consider to reference the following article: 

#### Automated Environmental Mapping with Behavior Trees

```
@incollection{rosas2022automated,
  title={Automated Environmental Mapping with Behavior Trees},
  author={Rosas, Francisco Garcia and Hoeller, Frank and Schneider, Frank E},
  booktitle={Advances in Intelligent Systems Research and Innovation},
  pages={61--85},
  year={2022},
  publisher={Springer}
}
```

## Contributors

```
Francisco J. Garcia R.
francisco.garcia.rosas@fkie.fraunhofer.de

Frank Höller
frank.hoeller@fkie.fraunhofer.de

Menaka Naazare 
menaka.naazare@fkie.fraunhofer.de
```
