// Copyright 2022 Fraunhofer FKIE - All Rights Reserved
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef _SENSOR_SIMULATOR_H_
#define _SENSOR_SIMULATOR_H_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>

#include "fkie_measurement_sensor_simulator/SourceDescription.hpp"
#include <fkie_measurement_msgs/Measurement.h>
#include <fkie_measurement_msgs/MeasurementArray.h>

class SensorSimulator
{
public:
  std::vector<SourceDescription> sources;

  std::string global_frame = "";
  std::string sensor_frame = "";
  std::string topic_measurement = "measurement";
  std::string topic_sensor_array = "sensor_array";
  double rate = 1.0;
  double marker_size = 0.2;
  int utm_zone_number = 0;
  std::string utm_zone_letter = "";

  // measurement properties
  std::string unique_serial_id = "";
  std::string manufacturer_device_name = "";
  std::string device_classification = "";

  // measurement value properties
  std::string sensor_name = "";
  std::string sensor_source_type = "";
  std::string sensor_unit = "";
  double random_factor = 0.1;

protected:
  tf::TransformListener tf_listener;
  tf::StampedTransform tf_world_sensor;
  ros::Publisher measurement_pub;
  ros::Publisher measurement_array_pub;
  ros::Publisher sensor_array_pub;
  ros::Publisher marker_location_pub;
  visualization_msgs::MarkerArray marker_locations;

  PositionGrid current_sensor_position;

public:
  SensorSimulator();

  ~SensorSimulator();

  /**
   * @brief Spin sensor simulation methods
   */
  void spin();

  /**
   * @brief Compute current robot position based on TF between [global_frame] and [sensor_frame]
   */
  void updateCurrentSensorPosition();

  /**
   * @brief Computes the accumulated sensor measurement assuming that sensor is located on [current_sensor_position]
   */
  double computeMeasurementFromSources() const;

  /**
   * @brief Publish  current measurements using [measurement_msgs_fkie]
   */
  void publishMeasurement(const double measurement) const;

  /**
   * @brief Publish the location of sensors using RVIZ markers
   */
  void publishSourceLocations();

  /**
   * @brief Computes euler distance between points [p1] and [p2]
   */
  double euclideanDistance(const PositionGrid &p1, const PositionGrid &p2) const;

  /**
   * @brief Read a ROS parameter
   */
  template <typename T>
  inline bool getROSParameter(std::string name, T &param, bool print = true)
  {
    const T default_value = param;
    bool r = ros::NodeHandle("~").param<T>(name, param, default_value);
    if (print)
      ROS_INFO_STREAM(name << ": [" << param << "]");
    return r;
  }
};

#endif /* _SENSOR_SIMULATOR_H_ */
