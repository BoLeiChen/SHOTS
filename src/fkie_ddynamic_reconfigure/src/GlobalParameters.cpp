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

/*
 * GlobalParameters.cpp
 *
 *  Created on: Mar 21, 2021
 *      Author: hoeller, garcia
 * 
 *  simplifly usage of ddynamic_reconfigure, adds singleton mechanism
 */


#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

namespace fkie_ddynamic_reconfigure 
{
    void paramCbInt(int& var, std::string s, int new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from %d to %d", s.c_str(), var, new_value );
      var = new_value;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbDouble(double& var, std::string s, double new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from %f to %f", s.c_str(), var, new_value );
      var = new_value;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbStr(std::string& var, std::string s, std::string new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from '%s' to '%s'", s.c_str(), var.c_str(), new_value.c_str() );
      var = new_value;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbBool(bool& var, std::string s, bool new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from '%d' to '%d'", s.c_str(), var, new_value );
      var = new_value;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbInt_boolOut(int& var, std::string s, bool& bool_out, int new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from %d to %d", s.c_str(), var, new_value );
      var = new_value;
      bool_out = true;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbDouble_boolOut(double& var, std::string s, bool& bool_out, double new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from %f to %f", s.c_str(), var, new_value );
      var = new_value;
      bool_out = true;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbStr_boolOut(std::string& var, std::string s, bool& bool_out, std::string new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from '%s' to '%s'", s.c_str(), var.c_str(), new_value.c_str() );
      var = new_value;
      bool_out = true;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }
    void paramCbBool_boolOut(bool& var, std::string s, bool& bool_out, bool new_value)
    {
      ROS_INFO_NAMED( "GlobalParams",  "param '%s' modified from '%d' to '%d'", s.c_str(), var, new_value );
      var = new_value;
      bool_out = true;
      ros::NodeHandle( "~" ).setParam( s, var );                                              \
    }

}
