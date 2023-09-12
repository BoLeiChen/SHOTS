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
#include <ros/ros.h>

int main(int argc, char** argv)
{
  // Initialize node and publisher.
  ros::init(argc, argv, "occupancy_grid_to_point_cloud");

  OccupancyGridToPointCloud og_to_pc;

  ros::Rate r(og_to_pc.params->spin_rate);

  while (ros::ok())
  {
    og_to_pc.spin();

    r.sleep();
    ros::spinOnce();
  }

  return 0;
}