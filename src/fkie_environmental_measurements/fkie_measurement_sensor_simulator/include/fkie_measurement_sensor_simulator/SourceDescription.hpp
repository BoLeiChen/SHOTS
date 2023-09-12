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

#ifndef SURFACE_DESCRIPTION_H_
#define SURFACE_DESCRIPTION_H_

#include <iostream>
#include <unordered_map>
#include <map>
#include <deque>
#include <string>
#include <algorithm>

#include <ros/ros.h>

#include "fkie_measurement_sensor_simulator/structs.hpp"

class SourceDescription
{
public:
  std::string name;
  std::string function;
  double intensity;
  PositionGrid position;

  double linear_alpha;
  double exponential_decay_rate;
  std_msgs::ColorRGBA color;
  std_msgs::ColorRGBA color_text;

  SourceDescription(){};

  [[nodiscard]] static bool updateSourceDescriptions(std::vector<SourceDescription>& sources)
  {
    sources.clear();
    ros::NodeHandle private_node = ros::NodeHandle("~");

    int counter = 0;
    std::string param_name = "sources/source_" + std::to_string(counter);

    while (private_node.hasParam(param_name + "/name"))
    {
      SourceDescription sd;
      std::vector<float> color_rgba;
      private_node.param<std::string>(param_name + "/name", sd.name, std::string(""));
      private_node.param<double>(param_name + "/intensity", sd.intensity, 0.0);
      private_node.param<double>(param_name + "/x", sd.position.x, 0.0);
      private_node.param<double>(param_name + "/y", sd.position.y, 0.0);
      private_node.param<double>(param_name + "/z", sd.position.z, 0.0);

      private_node.param<std::string>(param_name + "/function", sd.function, std::string(""));
      private_node.param<double>(param_name + "/linear_alpha", sd.linear_alpha, 0.0);
      private_node.param<double>(param_name + "/exponential_decay_rate", sd.exponential_decay_rate, 0.0);

      private_node.param<std::vector<float>>(param_name + "/color_rgba", color_rgba, std::vector<float>());
      private_node.param<double>(param_name + "/z", sd.position.z, 0.0);
      if (color_rgba.size() > 0)
      {
        sd.color.r = color_rgba[0];
        sd.color.g = color_rgba[1];
        sd.color.b = color_rgba[2];
        sd.color.a = color_rgba[3];
      }

      private_node.param<std::vector<float>>(param_name + "/text_color_rgba", color_rgba, std::vector<float>());
      if (color_rgba.size() > 0)
      {
        sd.color_text.r = color_rgba[0];
        sd.color_text.g = color_rgba[1];
        sd.color_text.b = color_rgba[2];
        sd.color_text.a = color_rgba[3];
      }

      sources.push_back(sd);
      counter++;
      param_name = "sources/source_" + std::to_string(counter);
    }

    if (counter == 0)  // no parameters were found
      return false;

    ROS_INFO_STREAM("Total sources: " << sources.size());

    return true;
  }
};

#endif /* SURFACE_DESCRIPTION_H_ */
