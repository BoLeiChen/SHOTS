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

#include "OccupancyGridToPointCloud.h"

OccupancyGridToPointCloud::OccupancyGridToPointCloud()
{
  params = &OccupancyGridToPointCloudParameters::getInstance();

  ros::NodeHandle nh;
  gridMapObj = grid_map::GridMap({ occupancyLayerName });  // Initiates Grid map

  gridMapPublisher = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1, true);
  pointCloudPublisher = nh.advertise<sensor_msgs::PointCloud2>("point_cloud", 1, true);

  occupancySubscriber = nh.subscribe(occupancyLayerName, 1, &OccupancyGridToPointCloud::occupancyCallback, this);
}

OccupancyGridToPointCloud::~OccupancyGridToPointCloud()
{
  occupancySubscriber.shutdown();
  gridMapPublisher.shutdown();
  pointCloudPublisher.shutdown();
};

bool OccupancyGridToPointCloud::spin()
{
  // If no new occupancy grid message has been received, return false.
  if (!isOccupancyReady)
    return false;

  // No nothing if no subscribers
  if (pointCloudPublisher.getNumSubscribers() == 0)
    return false;

  if (!addOccupancyToGridMap())
    return false;

  if (!addImageLayerToGridMap())
    return false;

  publishPointCloud();

  publishGridMap();

  // reset and wait for the next message to process
  isOccupancyReady = false;

  return true;
}

void OccupancyGridToPointCloud::occupancyCallback(const nav_msgs::OccupancyGrid& msg)
{
  const std::lock_guard<std::mutex> lock(mutexGridMap);
  occupancyMapCurrent = msg;
  isOccupancyReady = true;  // first message arrives, checker ready
}

void OccupancyGridToPointCloud::publishGridMap()
{
  grid_map_msgs::GridMap mapMessage;

  {
    const std::lock_guard<std::mutex> lock(mutexGridMap);
    grid_map::GridMapRosConverter::toMessage(gridMapObj, mapMessage);
  }

  gridMapPublisher.publish(mapMessage);
}

void OccupancyGridToPointCloud::publishPointCloud()
{
  sensor_msgs::PointCloud2 pointCloud;
  toPointCloud(gridMapObj, heightLayerName, occupancyLayerName, pointCloud);
  pointCloudPublisher.publish(pointCloud);
};

bool OccupancyGridToPointCloud::addOccupancyToGridMap()
{
  mapResolution = occupancyMapCurrent.info.resolution;
  widthMap = occupancyMapCurrent.info.width;
  heightMap = occupancyMapCurrent.info.height;
  mapFrame = occupancyMapCurrent.header.frame_id;

  // Check if Occupancy is valid
  if (mapResolution > 0 && widthMap > 0 && heightMap > 0 && occupancyMapCurrent.data.size() > 0)
  {
    const std::lock_guard<std::mutex> lock(mutexGridMap);

    gridMapObj.clearBasic();
    grid_map::GridMapRosConverter::fromOccupancyGrid(occupancyMapCurrent, occupancyLayerName, gridMapObj);

    // Check if conversion works
    if (gridMapObj.getLength().x() > 0.0 && gridMapObj.getLength().y() > 0.0)
    {
      gridMapObj.setFrameId(mapFrame);
      return true;
    }
    else
    {
      ROS_ERROR(
          "Fail conversion of OccupancyGrid into gridMap: gridMapObj.getLength().x() > 0.0 && "
          "gridMapObj.getLength().y() > 0.0");
      return false;
    }
  }
  else
  {
    ROS_INFO("Map not valid, check mapResolution > 0 && widthMap > 0 && heightMap > 0");
    return false;
  }
}

bool OccupancyGridToPointCloud::addImageLayerToGridMap()
{
  {
    const std::lock_guard<std::mutex> lock(mutexGridMap);

    // convert current gridmap to OpenCV image
    currentMapImage.release();
    OccupancyGridToPointCloud::toImage<unsigned char, 4>(gridMapObj, occupancyLayerName, CV_8UC4,
                                                         params->lower_threshold_image_map,
                                                         params->upper_threshold_image_map, currentMapImage, 255);
  }

  // convert current map to gray scale
  cv::cvtColor(currentMapImage, mapGray, CV_BGR2GRAY);

  // Apply Gaussian blur
  if (params->blur_radius % 2 == 0)
  {
    params->blur_radius -= 1;
    ROS_INFO_STREAM("ContourWallFollower: blur_radius must be odd number, new value: " << params->blur_radius);
  }

  // OpenCV operations
  try
  {
    cv::GaussianBlur(mapGray, mapGray, cv::Size(params->blur_radius, params->blur_radius), 0.0, 0.0);
    bw = (mapGray > params->threshold_image);

    // Dilate map (fill internal holes)
    element = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                        cv::Size(2 * params->erosion_size + 1, 2 * params->erosion_size + 1),
                                        cv::Point(params->erosion_size, params->erosion_size));

    cv::dilate(bw, bw, element);

    // Find contours
    cv::findContours(bw, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
  }
  catch (cv::Exception& e)
  {
    ROS_ERROR_STREAM("updateImage error: " << e.what());
    return false;
  }

  cv::Mat outputImage = cv::Mat::zeros(bw.size(), CV_8UC3);

  // draw contours with areas bigger than a threshold
  for (size_t i = 0; i < contours.size(); i++)
  {
    //  Find the area of contour
    // if (hierarchy[i][3] > -1)   // only consider contours with parent
    {
      double a = cv::contourArea(contours[i], false);
      if (a >= params->min_area_contour)
      {
        cv::Scalar color = cv::Scalar(255, 247, 204);
        drawContours(outputImage, contours, i, color, 2, 8, hierarchy, 0, cv::Point());
      }
    }
  }

  // valid for encoding: CV_8UC3
  // more examples:
  // https://github.com/ANYbotics/grid_map/blob/761e11b7453fba013b6e2a86d83cb9a790385fec/grid_map_ros/src/GridMapRosConverter.cpp#L345
  {
    const std::lock_guard<std::mutex> lock(mutexGridMap);

    grid_map::GridMapCvConverter::addColorLayerFromImage<unsigned char, 3>(outputImage, heightLayerName, gridMapObj);

    // static bool addLayerFromImage(const sensor_msgs::Image &image, const std::string &layer,
    //                               grid_map::GridMap &gridMap, const float lowerValue = 0.0,
    //                               const float upperValue = 1.0, const double alphaThreshold = 0.5)
    const float lowerValue = 0.0;
    const float upperValue = 1.0;
    const double alphaThreshold = 0.5;
    grid_map::GridMapCvConverter::addLayerFromImage<unsigned char, 3>(outputImage, postProcessedLayerName, gridMapObj,
                                                                      lowerValue, upperValue, alphaThreshold);
  }

  return true;
}

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
bool OccupancyGridToPointCloud::toImage(const grid_map::GridMap& gridMap, const std::string& layer, const int encoding,
                                        const float lowerValue, const float upperValue, cv::Mat& image,
                                        float unknown_value)
{
  // Initialize image.
  if (gridMap.getSize()(0) > 0 && gridMap.getSize()(1) > 0)
  {
    image = cv::Mat::ones(gridMap.getSize()(0), gridMap.getSize()(1), encoding);
  }
  else
  {
    std::cerr << "Invalid grid map?" << std::endl;
    return false;
  }

  // Get max image value.
  Type_ imageMax;
  if (std::is_same<Type_, float>::value || std::is_same<Type_, double>::value)
  {
    imageMax = 1.0;
  }
  else if (std::is_same<Type_, unsigned short>::value || std::is_same<Type_, unsigned char>::value)
  {
    imageMax = (Type_)std::numeric_limits<Type_>::max();
  }
  else
  {
    std::cerr << "This image type is not supported." << std::endl;
    return false;
  }

  // Clamp outliers.
  grid_map::GridMap map = gridMap;
  map.get(layer) = map.get(layer).unaryExpr(grid_map::Clamp<float>(lowerValue, upperValue));
  const grid_map::Matrix& data = map[layer];

  // Convert to image.
  bool isColor = false;
  if (image.channels() >= 3)
    isColor = true;
  bool hasAlpha = false;
  if (image.channels() >= 4)
    hasAlpha = true;

  for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator)
  {
    const grid_map::Index index(*iterator);

    if (std::isfinite(data(index(0), index(1))))
    {
      const float& value = data(index(0), index(1));
      const Type_ imageValue = (Type_)(((value - lowerValue) / (upperValue - lowerValue)) * (float)imageMax);
      const grid_map::Index imageIndex(iterator.getUnwrappedIndex());

      unsigned int channel = 0;
      image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = imageValue;

      if (isColor)
      {
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageValue;
      }
      if (hasAlpha)
      {
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = imageMax;
      }
    }
    else
    {
      // add constant value to unknown cells
      unsigned int channel = 0;
      const grid_map::Index imageIndex(iterator.getUnwrappedIndex());
      image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[channel] = unknown_value;

      if (isColor)
      {
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = unknown_value;
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = unknown_value;
      }
      if (hasAlpha)
      {
        image.at<cv::Vec<Type_, NChannels_>>(imageIndex(0), imageIndex(1))[++channel] = unknown_value;
      }
    }
  }

  return true;
};

void OccupancyGridToPointCloud::toPointCloud(const grid_map::GridMap& gridMap, const std::string heightLayer,
                                             const std::string baseLayer, sensor_msgs::PointCloud2& cloud)
{
  // Header.
  cloud.header.frame_id = gridMap.getFrameId();
  cloud.header.stamp = ros::Time::now();
  // cloud.header.stamp.fromNSec(gridMap.getTimestamp());

  sensor_msgs::PointCloud2Modifier mod(cloud);
  mod.setPointCloud2FieldsByString(1, "xyz");
  int max_z_layers =
      (int)((params->point_cloud_max_z - params->point_cloud_min_z) / params->point_cloud_step_size_z) + 1;
  mod.resize(gridMap.getSize().prod() * max_z_layers);

  sensor_msgs::PointCloud2Iterator<float> xyz(cloud, "x");

  const grid_map::Matrix& dataBase = gridMap[baseLayer];

  const grid_map::Matrix& dataHeight = gridMap[heightLayer];

  std::size_t stored = 0;
  for (grid_map::GridMapIterator iterator(gridMap); !iterator.isPastEnd(); ++iterator)
  {
    grid_map::Position position;
    gridMap.getPosition(*iterator, position);
    const grid_map::Index index(*iterator);
    double valueBase = dataBase(index(0), index(1));
    double valueHeight = (double)dataHeight(index(0), index(1));

    // ignore unknown space
    if (std::isnan(valueBase) || !std::isfinite(valueHeight))
      continue;

    bool borderPoint = (valueHeight != 0.0);

    // create only one layer if the point does not correspond to a border, and the min_z is Zero
    if (!borderPoint && params->point_cloud_min_z == 0.0)
    {
      xyz[0] = (float)((double)(position.x()) + params->point_cloud_offset_x);
      xyz[1] = (float)((double)(position.y()) + params->point_cloud_offset_y);
      xyz[2] = (float)(0.0 + params->point_cloud_offset_z);

      ++xyz;
      ++stored;
      continue;
    }
    else if (borderPoint)
    {
      // create layers between [point_cloud_min_z] and [point_cloud_max_z]
      for (double new_z = params->point_cloud_min_z; new_z <= params->point_cloud_max_z;
           new_z += params->point_cloud_step_size_z)
      {
        xyz[0] = (float)((double)(position.x()) + params->point_cloud_offset_x);
        xyz[1] = (float)((double)(position.y()) + params->point_cloud_offset_y);
        xyz[2] = (float)(new_z + params->point_cloud_offset_z);

        ++xyz;
        ++stored;
      }
    }
  }
  mod.resize(stored);
}
