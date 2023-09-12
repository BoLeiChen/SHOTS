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

#include "MeasurementsToPointCloud.h"

MeasurementsToPointCloud::MeasurementsToPointCloud()
{
  ros::NodeHandle nh;
  params = &MeasurementsToPointCloudParameters::getInstance();

  ac = std::make_unique<ClientInterpolation>(params->topic_request_interpolation, true);
  ROS_INFO_STREAM("MeasurementsToPointCloud: Waiting for action server [" << params->topic_request_interpolation
                                                                          << "]");
  ac->waitForServer();

  current_max_mean_value = params->initial_max_mean_value;

  white.r = 1.0;
  white.g = 1.0;
  white.b = 1.0;
  white.a = 1.0;

  black.r = 0.0;
  black.g = 0.0;
  black.b = 0.0;
  black.a = 1.0;

  red.r = 0.83;
  red.g = 0.2;
  red.b = 0.2;
  red.a = 1.0;

  green.r = 0.63;
  green.g = 0.87;
  green.b = 0.67;
  green.a = 1.0;

  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("color_point_cloud", 1, true);
  pointCloudSubscriber = nh.subscribe("point_cloud", 1, &MeasurementsToPointCloud::pointCloudCallback, this);
}

MeasurementsToPointCloud::~MeasurementsToPointCloud()
{
  pointCloudSubscriber.shutdown();
  ac->cancelAllGoals();
};

bool MeasurementsToPointCloud::spin()
{
  // If no new occupancy grid message has been received, return false.
  if (!isPointCloudReady)
    return false;

  // No nothing if no subscribers
  if (pointCloudPublisher.getNumSubscribers() == 0)
    return false;

  if (!updateMeasurementGrid())
    return false;

  if (!computePointCloud())
    return false;

  return true;
}

void MeasurementsToPointCloud::pointCloudCallback(const sensor_msgs::PointCloud2& msg)
{
  const std::lock_guard<std::mutex> lock(mutexPointCloud);
  currentPointCloud = msg;

  isPointCloudReady = true;
}

bool MeasurementsToPointCloud::updateMeasurementGrid()
{
  // fill point cloud points into request
  {
    const std::lock_guard<std::mutex> lock(mutexPointCloud);

    // https://answers.ros.org/question/11556/datatype-to-access-pointcloud2/
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(currentPointCloud, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(currentPointCloud, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(currentPointCloud, "z");

    geometry_msgs::PointStamped ps;
    ps.header = currentPointCloud.header;

    currentGoalAction = fkie_measurement_msgs::RequestGaussianInterpolationGoal();

    for (; (iter_x != iter_x.end()) && (iter_y != iter_y.end()) && (iter_z != iter_z.end());
         ++iter_x, ++iter_y, ++iter_z)
    {
      ps.point.x = *iter_x;
      ps.point.y = *iter_y;
      ps.point.z = *iter_z;
      currentGoalAction.interpolation_points.push_back(ps);
    }
  }

  if (currentGoalAction.interpolation_points.size() < 2)
  {
    ROS_ERROR_STREAM("Too few points for interpolation");
    return false;
  }

  {
    const std::lock_guard<std::mutex> lock(mutexInterpolation);

    std::vector<std::string> measurement_type_list;
    measurement_type_list.push_back(params->source_type);
    currentGoalAction.measurement_type_list = measurement_type_list;

    ac->sendGoal(currentGoalAction, boost::bind(&MeasurementsToPointCloud::doneCallback, this, _1, _2),
                 boost::bind(&MeasurementsToPointCloud::activeCallback, this),
                 boost::bind(&MeasurementsToPointCloud::feedbackCallback, this, _1));

    ac->waitForResult();
  }

  if (resultInterpolation.mean.size() == 0)
  {
    ROS_ERROR_STREAM("Could not compute interpolation, check output from measurement server node.");
    return false;
  }

  return true;
}

bool MeasurementsToPointCloud::computePointCloud()
{
  // Header
  sensor_msgs::PointCloud2 outputCloud;
  outputCloud.header.frame_id = currentPointCloud.header.frame_id;
  outputCloud.header.stamp = ros::Time::now();

  sensor_msgs::PointCloud2Modifier mod(outputCloud);
  {
    const std::lock_guard<std::mutex> lock(mutexPointCloud);
    mod.setPointCloud2FieldsByString(2, "xyz", "rgb");
    mod.resize(currentPointCloud.row_step * currentPointCloud.height);
  }

  // sensor_msgs::PointCloud2Iterator<float> cloud_iter(outputCloud, "x");

  sensor_msgs::PointCloud2Iterator<float> out_x(outputCloud, "x");
  sensor_msgs::PointCloud2Iterator<float> out_y(outputCloud, "y");
  sensor_msgs::PointCloud2Iterator<float> out_z(outputCloud, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_r(outputCloud, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_g(outputCloud, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> out_b(outputCloud, "b");

  size_t stored = 0;
  {
    const std::lock_guard<std::mutex> lock(mutexInterpolation);
    for (size_t i_cell = 0; i_cell < currentGoalAction.interpolation_points.size(); ++i_cell)
    {
      geometry_msgs::Point p = currentGoalAction.interpolation_points[i_cell].point;

      *out_x = p.x;
      *out_y = p.y;
      *out_z = p.z;

      // normalize mean between 0 and max
      if (resultInterpolation.mean[i_cell] > current_max_mean_value)
        current_max_mean_value = resultInterpolation.mean[i_cell];

      if (resultInterpolation.variance[i_cell] > current_max_variance_value)
        current_max_variance_value = resultInterpolation.variance[i_cell];

      std_msgs::ColorRGBA color;

      if (resultInterpolation.variance[i_cell] < params->minimum_variance)
      {
        // color the point cloud with mean
        double normalized_mean = (resultInterpolation.mean[i_cell] / current_max_mean_value);
        color = interpolateColor(green, red, normalized_mean);
      }
      else
      {
        // use gray color based on variance
        double normalized_variance = (resultInterpolation.variance[i_cell] / current_max_variance_value);
        color = interpolateColor(white, black, normalized_variance);
      }

      *out_r = (int)(color.r * 255);
      *out_g = (int)(color.g * 255);
      *out_b = (int)(color.b * 255);

      ++out_x;
      ++out_y;
      ++out_z;
      ++out_r;
      ++out_g;
      ++out_b;
      ++stored;
    }
  }
  mod.resize(stored);

  pointCloudPublisher.publish(outputCloud);
  return true;
}

void MeasurementsToPointCloud::doneCallback(
    const actionlib::SimpleClientGoalState& state,
    const fkie_measurement_msgs::RequestGaussianInterpolationResultConstPtr& result)
{
  if (state != actionlib::SimpleClientGoalState::LOST && result != NULL && result->statistics.size() > 0)
  {
    resultInterpolation = result->statistics[0];
  }
  else
  {
    resultInterpolation = fkie_measurement_msgs::MeasurementGaussianStatistics();
    ROS_ERROR_STREAM("Transition to LOST. Check if Action Server [" << params->topic_request_interpolation
                                                                    << "] is running.");
  }
}

void MeasurementsToPointCloud::activeCallback()
{
  ROS_DEBUG_STREAM("activeCallback Not Implemented");
};

void MeasurementsToPointCloud::feedbackCallback(
    const fkie_measurement_msgs::RequestGaussianInterpolationFeedbackConstPtr& feedback)
{
  ROS_DEBUG_STREAM("feedbackCallback Not Implemented");
};
