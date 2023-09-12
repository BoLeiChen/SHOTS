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

#ifndef OCCUPANCY_GRID_TO_POINT_CLOUD_PARAMETERS_H_
#define OCCUPANCY_GRID_TO_POINT_CLOUD_PARAMETERS_H_

#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

class OccupancyGridToPointCloudParameters
  : public fkie_ddynamic_reconfigure::GlobalParameters<OccupancyGridToPointCloudParameters>
{
protected:
  static OccupancyGridToPointCloudParameters* globalPtr;

public:
  // ros parameters
  double spin_rate = 1.0;
  double lower_threshold_image_map = 10.0;
  double upper_threshold_image_map = 50.0;
  int blur_radius = 3;
  double erosion_size = 1.0;
  double threshold_image = 50.0;
  double min_area_contour = 0.0;

  double point_cloud_min_z = 0.3;
  double point_cloud_max_z = 1.5;
  double point_cloud_step_size_z = 0.3;
  double point_cloud_offset_x = 0.0;
  double point_cloud_offset_y = 0.0;
  double point_cloud_offset_z = -0.15;

  OccupancyGridToPointCloudParameters()
  {
    dynreconfRegDoubleGroup(spin_rate, "spin rate", 1.0, 20.0, "PostProcessing");
    dynreconfRegDoubleGroup(lower_threshold_image_map, "Lower occupancy grid value for post-processing", 0.0, 255.0,
                            "PostProcessing");
    dynreconfRegDoubleGroup(upper_threshold_image_map, "Upper occupancy grid value for post-processing", 0.0, 255.0,
                            "PostProcessing");
    dynreconfRegIntGroup(blur_radius, "Radius for blurring the occupancy grid", 0, 30, "PostProcessing");
    dynreconfRegDoubleGroup(erosion_size, "Erosion size for post-processing the occupancy grid", 0.0, 30.0,
                            "PostProcessing");
    dynreconfRegDoubleGroup(threshold_image, "threshold_image", 0.0, 100.0, "PostProcessing");
    dynreconfRegDoubleGroup(min_area_contour, "Minimun area of a contour to be cosireded for post-processing", 0.0,
                            300.0, "PostProcessing");

    dynreconfRegDoubleGroup(point_cloud_min_z, "Lower Z value", -50.0, 50.0, "PointCloud");
    dynreconfRegDoubleGroup(point_cloud_max_z, "Higher Z value", -500.0, 50.0, "PointCloud");
    dynreconfRegDoubleGroup(point_cloud_step_size_z, "Step size", 0.1, 5.0, "PointCloud");
    dynreconfRegDoubleGroup(point_cloud_offset_x, "Offset for X", -10.0, 10.0, "PointCloud");
    dynreconfRegDoubleGroup(point_cloud_offset_y, "Offset for Y", -10.0, 10.0, "PointCloud");
    dynreconfRegDoubleGroup(point_cloud_offset_z, "Offset for Z", -10.0, 10.0, "PointCloud");

    ddr->publishServicesTopics();
  }

  template <typename T>
  bool getROSUnsupportedTypeParameter(std::string name, T& param)
  {
    ros::NodeHandle private_node = ros::NodeHandle("~");
    const T default_value = param;
    bool r = private_node.param<T>(name, param, default_value);
    return r;
  }
};

#endif /* OCCUPANCY_GRID_TO_POINT_CLOUD_PARAMETERS_H_ */
