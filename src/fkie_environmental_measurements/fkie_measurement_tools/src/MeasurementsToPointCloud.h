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

#ifndef MEASUREMENTS_TO_POINT_CLOUD
#define MEASUREMENTS_TO_POINT_CLOUD

#include "MeasurementsToPointCloudParameters.hpp"
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/ColorRGBA.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

#include <actionlib/client/simple_action_client.h>
#include <fkie_measurement_msgs/RequestGaussianInterpolationAction.h>

typedef actionlib::SimpleActionClient<fkie_measurement_msgs::RequestGaussianInterpolationAction>
    ClientInterpolation;

class MeasurementsToPointCloud
{
public:
  MeasurementsToPointCloudParameters* params = NULL;

private:
  std::mutex mutexPointCloud;
  std::mutex mutexInterpolation;

  std::unique_ptr<ClientInterpolation> ac;
  fkie_measurement_msgs::MeasurementGaussianStatistics resultInterpolation;
  fkie_measurement_msgs::RequestGaussianInterpolationGoal currentGoalAction;

  ros::Subscriber pointCloudSubscriber;
  ros::Publisher pointCloudPublisher;

  bool isPointCloudReady = false;
  double current_max_mean_value = 0.0;
  double current_max_variance_value = 0.0;

  sensor_msgs::PointCloud2 currentPointCloud;

public:
  MeasurementsToPointCloud();
  ~MeasurementsToPointCloud();
  bool spin();

private:
  void pointCloudCallback(const sensor_msgs::PointCloud2& msg);

  /**
   * @brief connects to measurement server and request interpolation results
   */
  [[nodiscard]] bool updateMeasurementGrid();

  /**
   * @brief Take current point cloud, add a new color layer based on measurements
   */
  [[nodiscard]] bool computePointCloud();

  // request interplation messages
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const fkie_measurement_msgs::RequestGaussianInterpolationResultConstPtr& result);
  void activeCallback();
  void
  feedbackCallback(const fkie_measurement_msgs::RequestGaussianInterpolationFeedbackConstPtr& feedback);

  std_msgs::ColorRGBA interpolateColor(std_msgs::ColorRGBA color_a, std_msgs::ColorRGBA color_b, double fraction)
  {
    std_msgs::ColorRGBA c;
    c.r = color_a.r + (color_b.r - color_a.r) * fraction;
    c.g = color_a.g + (color_b.g - color_a.g) * fraction;
    c.b = color_a.b + (color_b.b - color_a.b) * fraction;
    c.a = color_a.a + (color_b.a - color_a.a) * fraction;
    return c;
  };

  std_msgs::ColorRGBA white;
  std_msgs::ColorRGBA black;
  std_msgs::ColorRGBA red;
  std_msgs::ColorRGBA green;
};

#endif /* MEASUREMENTS_TO_POINT_CLOUD */