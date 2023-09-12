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

int main(int argc, char** argv)
{
  ros::init(argc, argv, "measurements_to_point_cloud");
  ros::NodeHandle private_node = ros::NodeHandle("~");
  ros::NodeHandle public_node = ros::NodeHandle("");

  MeasurementsToPointCloud m_to_pc;

  ros::Rate r(m_to_pc.params->spin_rate);

  while (ros::ok())
  {
    m_to_pc.spin();

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}