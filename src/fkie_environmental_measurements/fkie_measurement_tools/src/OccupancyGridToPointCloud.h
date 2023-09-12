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

#ifndef OCCUPANCY_GRID_TO_POINT_CLOUD
#define OCCUPANCY_GRID_TO_POINT_CLOUD

#include "OccupancyGridToPointCloudParameters.hpp"
#include <grid_map_cv/grid_map_cv.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <mutex>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <string>

class OccupancyGridToPointCloud
{
public:
  OccupancyGridToPointCloudParameters* params = NULL;

private:
  std::mutex mutexGridMap;

  ros::Subscriber occupancySubscriber;
  ros::Publisher gridMapPublisher;
  ros::Publisher pointCloudPublisher;

  bool isOccupancyReady = false;
  grid_map::GridMap gridMapObj;
  nav_msgs::OccupancyGrid occupancyMapCurrent;
  std::string mapFrame;
  float mapResolution = 0.0, widthMap = 0.0, heightMap = 0.0;

  // OpenCV objects
  cv::Mat currentMapImage;
  cv::Mat mapGray, bw, element;
  std::vector<std::vector<cv::Point>> contours;
  std::vector<cv::Vec4i> hierarchy;

  const std::string occupancyLayerName = "occupancy";
  const std::string postProcessedLayerName = "postprocessed";
  const std::string heightLayerName = "height";

public:
  OccupancyGridToPointCloud();

  ~OccupancyGridToPointCloud();

  bool spin();

private:
  void occupancyCallback(const nav_msgs::OccupancyGrid& msg);

  /**
   * @brief Publish current grid map object
   */
  void publishGridMap();

  /**
   * @brief Publish a point cloud generated from the current grid map object
   */
  void publishPointCloud();

  /**
   * @brief Creates a new layer in the gridmap object, based on current occupancy grid
   */
  [[nodiscard]] bool addOccupancyToGridMap();

  /**
   * @brief Creates a new layer in the gridmap object, based on the current post-processed image
   */
  [[nodiscard]] bool addImageLayerToGridMap();

  /*!
   * based on: grid_map_cv/include/grid_map_cv/GridMapCvConverter.hpp
   * Creates a cv mat from a grid map layer.
   * @param[in] grid map to be added.
   * @param[in] layer the layer that is converted to the image.
   * @param[in] encoding the desired encoding of the image.
   * @param[in] lowerValue the value of the layer corresponding to black image pixels.
   * @param[in] upperValue the value of the layer corresponding to white image pixels.
   * @param[out] image the image to be populated.
   * @return true if successful, false otherwise.
   */
  template <typename Type_, int NChannels_>
  static bool toImage(const grid_map::GridMap& gridMap, const std::string& layer, const int encoding,
                      const float lowerValue, const float upperValue, cv::Mat& image, float unknown_value);

  void toPointCloud(const grid_map::GridMap& gridMap, const std::string heightLayer, const std::string baseLayer,
                    sensor_msgs::PointCloud2& cloud);
};

#endif /* OCCUPANCY_GRID_TO_POINT_CLOUD */