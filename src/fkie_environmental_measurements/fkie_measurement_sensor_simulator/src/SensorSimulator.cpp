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

#include "SensorSimulator.h"

SensorSimulator::SensorSimulator()
{
  // Simulator properties
  getROSParameter<std::string>("global_frame", global_frame);
  getROSParameter<std::string>("sensor_frame", sensor_frame);
  getROSParameter<double>("rate", rate);
  getROSParameter<std::string>("topic_measurement", topic_measurement);
  getROSParameter<std::string>("topic_sensor_array", topic_sensor_array);
  getROSParameter<double>("marker_size", marker_size);

  // Measurement properties
  getROSParameter<std::string>("unique_serial_id", unique_serial_id);
  getROSParameter<std::string>("manufacturer_device_name", manufacturer_device_name);
  getROSParameter<std::string>("device_classification", device_classification);

  // Measurement value properties
  getROSParameter<std::string>("sensor_name", sensor_name);
  getROSParameter<std::string>("sensor_source_type", sensor_source_type);
  getROSParameter<std::string>("sensor_unit", sensor_unit);

  getROSParameter<double>("random_factor", random_factor);
  getROSParameter<int>("utm_zone_number", utm_zone_number);
  getROSParameter<std::string>("utm_zone_letter", utm_zone_letter);

  if (global_frame.empty() || sensor_frame.empty() || unique_serial_id.empty() || manufacturer_device_name.empty() ||
      device_classification.empty() || sensor_name.empty() || sensor_source_type.empty() || sensor_unit.empty())
  {
    ROS_ERROR_STREAM("Invalid empty parameter, please check input parameters!");
    return;
  }

  // update parameters
  if (!SourceDescription::updateSourceDescriptions(sources))
  {
    ROS_ERROR_STREAM("Could not update source descriptions!");
    return;
  }

  // initialize publishers
  ros::NodeHandle nh = ros::NodeHandle("");
  measurement_pub = nh.advertise<fkie_measurement_msgs::Measurement>(topic_measurement, 10, false);
  measurement_array_pub =
      nh.advertise<fkie_measurement_msgs::MeasurementArray>(topic_measurement + "_array", 10, false);
  sensor_array_pub =
      nh.advertise<fkie_measurement_msgs::MeasurementArray>(topic_sensor_array, 10, true);
  marker_location_pub = nh.advertise<visualization_msgs::MarkerArray>("sensor_locations", 10, true);

  publishSourceLocations();

  spin();
}

SensorSimulator::~SensorSimulator()
{
  measurement_pub.shutdown();
  measurement_array_pub.shutdown();
  sensor_array_pub.shutdown();
  marker_location_pub.shutdown();
}

void SensorSimulator::spin()
{
  ros::Rate loop_rate(rate);

  while (ros::ok())
  {
    // Get current sensor position
    updateCurrentSensorPosition();
    ROS_DEBUG_STREAM("sensor_position: " << current_sensor_position.toString());

    // compute accumulated_intensity from all sensors
    double accumulated_intensity = computeMeasurementFromSources();
    ROS_DEBUG_STREAM("accumulated_intensity: " << accumulated_intensity);

    // publish message
    publishMeasurement(accumulated_intensity);

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void SensorSimulator::updateCurrentSensorPosition()
{
  // get tf between world_frame and path_frame
  while (ros::ok() && (!tf_listener.waitForTransform(global_frame, sensor_frame, ros::Time(), ros::Duration(0.5))))
  {
    ROS_INFO_STREAM("Wait for transform between [" << global_frame << "] and [" << sensor_frame << "]");
    ros::Duration(0.5).sleep();
  }
  tf_listener.lookupTransform(global_frame, sensor_frame, ros::Time(0), tf_world_sensor);

  // convert point to world frame
  tf::Stamped<tf::Point> pout_test, pin_test;
  pin_test.setX(0);
  pin_test.setY(0);
  pin_test.setZ(0);
  pout_test.setData(tf_world_sensor * pin_test);

  current_sensor_position = PositionGrid(pout_test.getX(), pout_test.getY(), pout_test.getZ());
}

double SensorSimulator::computeMeasurementFromSources() const
{
  float rfactor = 0.0;
  if (random_factor != 0.0) {
    rfactor = static_cast <float> (rand()) / static_cast <float> (RAND_MAX / random_factor);
  }
  double accumulated_intensity = 0.0;

  for (SourceDescription s : sources)
  {
    double distance = euclideanDistance(s.position, current_sensor_position);
    double m = 0.0;
    if (s.function == "linear" && std::abs(distance) > 0.0)
    {
      m = s.linear_alpha * (s.intensity / distance);
    }
    else if (s.function == "exponential" && std::abs(distance) > 0.0)
    {
      m = s.intensity * s.linear_alpha * std::exp(-s.exponential_decay_rate * distance);
    }
    else if (s.function == "inverse_squared")
    {
      m = s.intensity * (1.0 / std::pow(distance, 2));
    }
    else
    {
      ROS_WARN_STREAM("Unsupported function: [" << s.function << "]");
    }

    accumulated_intensity += m;
  }

  return accumulated_intensity + accumulated_intensity * rfactor;
}

void SensorSimulator::publishMeasurement(const double measurement) const
{
  fkie_measurement_msgs::Measurement m;

  // header:
  //   frame_id: TF frame to which the sensor is attached
  //   stamp: Time when this message was generated
  m.header.frame_id = sensor_frame;
  m.header.stamp = ros::Time::now();

  // Unique ID that identifies the device
  m.unique_serial_id = unique_serial_id;

  // Generic name assigned by the device manufacturer
  m.manufacturer_device_name = manufacturer_device_name;

  // classification that groups what the device is able to measure:
  //   e.g. chemical (C), biological (B), radiological (R), meteorologic (M), (W) WiFi etc...
  m.device_classification = device_classification;

  fkie_measurement_msgs::MeasurementValue v;
  v.begin = ros::Time::now();
  v.end = v.begin;
  v.sensor = sensor_name;
  v.source_type = sensor_source_type;
  v.unit = sensor_unit;
  v.value_single = measurement;

  m.values.push_back(v);

  measurement_pub.publish(m);

  // publish measurement array
  fkie_measurement_msgs::MeasurementLocated m_located;
  m_located.measurement = m;
  m_located.pose = current_sensor_position.toPoseStamped(global_frame);
  m_located.utm_zone_number = utm_zone_number;
  m_located.utm_zone_letter = utm_zone_letter;

  fkie_measurement_msgs::MeasurementArray m_array;
  m_array.header.stamp = ros::Time::now();
  m_array.full_history = false;
  m_array.located_measurements.push_back(m_located);
  measurement_array_pub.publish(m_array);
}

void SensorSimulator::publishSourceLocations()
{
  marker_locations.markers.clear();

  visualization_msgs::Marker m;
  m.header.frame_id = global_frame;
  m.header.stamp = ros::Time::now();
  m.id = 0;
  m.ns = "sources";
  m.action = visualization_msgs::Marker::ADD;
  m.type = visualization_msgs::Marker::SPHERE_LIST;
  m.scale.x = marker_size;
  m.scale.y = marker_size;
  m.scale.z = marker_size;
  m.lifetime = ros::Duration(0);
  m.pose.orientation.w = 1.0;

  fkie_measurement_msgs::MeasurementArray m_array;
  m_array.header.stamp = ros::Time::now();
  m_array.full_history = true;

  int counter_id = 1;
  for (SourceDescription s : sources)
  {
    geometry_msgs::Point pp;
    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = s.position.z;
    m.points.push_back(pp);
    m.colors.push_back(s.color);

    // add text description of source
    // std::string description = s.name + " (" + std::to_string((int)s.intensity) + ")";
    std::string description = s.name;

    visualization_msgs::Marker m_text;
    m_text.header.frame_id = global_frame;
    m_text.header.stamp = ros::Time::now();
    m_text.id = counter_id++;
    m_text.ns = "description";
    m_text.action = visualization_msgs::Marker::ADD;
    m_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m_text.scale.z = marker_size;
    m_text.lifetime = ros::Duration(0);
    m_text.pose.orientation.w = 1.0;
    m_text.text = description;
    m_text.pose.position.x = s.position.x;
    m_text.pose.position.y = s.position.y;
    m_text.pose.position.z = s.position.z + marker_size;
    m_text.color = s.color_text;
    marker_locations.markers.push_back(m_text);

    visualization_msgs::Marker m_line;
    m_line.header.frame_id = global_frame;
    m_line.header.stamp = ros::Time::now();
    m_line.id = counter_id++;
    m_line.ns = "ref_line";
    m_line.action = visualization_msgs::Marker::ADD;
    m_line.type = visualization_msgs::Marker::ARROW;
    m_line.scale.x = 0.2 * marker_size;
    m_line.scale.y = 0.2 * marker_size;
    m_line.scale.z = 0.0;
    m_line.lifetime = ros::Duration(0);
    m_line.pose.orientation.w = 1.0;

    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = 0.0;
    m_line.points.push_back(pp);

    pp.x = s.position.x;
    pp.y = s.position.y;
    pp.z = s.position.z;
    m_line.points.push_back(pp);

    m_line.color = s.color_text;
    marker_locations.markers.push_back(m_line);

    // create measurement array
    fkie_measurement_msgs::MeasurementLocated m_located;
    m_located.measurement.header.frame_id = global_frame;
    m_located.measurement.header.stamp = ros::Time::now();
    // Unique ID that identifies the device
    m_located.measurement.unique_serial_id = s.name;
    // Generic name assigned by the device manufacturer
    m_located.measurement.manufacturer_device_name = s.name;
    // classification that groups what the device is able to measure:
    //   e.g. chemical (C), biological (B), radiological (R), meteorologic (M), (W) WiFi etc...
    m_located.measurement.device_classification = device_classification;

    fkie_measurement_msgs::MeasurementValue v;
    v.begin = ros::Time::now();
    v.end = v.begin;
    v.sensor = s.name;
    v.source_type = sensor_source_type;
    v.unit = sensor_unit;
    v.value_single = s.intensity;
    m_located.measurement.values.push_back(v);

    // publish measurement array
    m_located.pose.header.frame_id = global_frame;
    m_located.pose.header.stamp = ros::Time::now();
    m_located.pose.pose.position.x = s.position.x;
    m_located.pose.pose.position.y = s.position.y;
    m_located.pose.pose.position.z = s.position.z + marker_size;
    m_located.utm_zone_number = utm_zone_number;
    m_located.utm_zone_letter = utm_zone_letter;
    m_array.located_measurements.push_back(m_located);
  }
  marker_locations.markers.push_back(m);
  marker_location_pub.publish(marker_locations);
  sensor_array_pub.publish(m_array);
}

double SensorSimulator::euclideanDistance(const PositionGrid &p1, const PositionGrid &p2) const
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}