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

#ifndef _SENSOR_SERVER_DESCRIPTION_H_
#define _SENSOR_SERVER_DESCRIPTION_H_

#include <iostream>
#include <unordered_map>
#include <map>
#include <deque>
#include <string>
#include <algorithm>

#include <ros/ros.h>

#include "fkie_3d_measurement_server/structs.hpp"

class SensorServerDescription
{
public:
  std::string name;
  std::string frame_id;

  SensorServerDescription(){};

  static bool updateSensorServerDescriptions(std::vector<SensorServerDescription>& sensors)
  {
    sensors.clear();
    ros::NodeHandle private_node = ros::NodeHandle("~");

    int counter = 0;
    std::string param_name = "sensors/sensor_" + std::to_string(counter);

    while (private_node.hasParam(param_name + "/name"))
    {
      SensorServerDescription sd;
      private_node.param<std::string>(param_name + "/name", sd.name, std::string(""));
      private_node.param<std::string>(param_name + "/frame_id", sd.frame_id, std::string(""));
      sensors.push_back(sd);
      counter++;
      param_name = "sensors/sensor_" + std::to_string(counter);
    }

    if (counter == 0)  // no parameters were found
      return false;

    ROS_INFO_STREAM("Total sensors: " << sensors.size());

    return true;
  }

  static std::string getFrameId(std::vector<SensorServerDescription>& sensors, std::string sensor_name)
  {
    for (SensorServerDescription s : sensors)
    {
      if (s.name == sensor_name)
        return s.frame_id;
    }

    return std::string();
  }
};

#endif /* _SENSOR_SERVER_DESCRIPTION_H_ */
