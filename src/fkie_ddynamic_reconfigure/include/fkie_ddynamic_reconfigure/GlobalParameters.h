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
 * GlobalParameters.h
 *
 *  Created on: Mar 21, 2021
 *      Author: hoeller, garcia
 * 
 *  simplifly usage of ddynamic_reconfigure, adds singleton mechanism
 */

#ifndef VIRTUALGLOBALPARAMETERS_H_
#define VIRTUALGLOBALPARAMETERS_H_

#include <math.h>
#include <string>
#include <boost/thread.hpp>
#include <boost/thread/condition.hpp>
#include <memory>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/console.h>
#include <fkie_ddynamic_reconfigure/ddynamic_reconfigure.h>

namespace fkie_ddynamic_reconfigure 
{
  #define dynreconfRegInt( name, desc, min, max )                                                                                                       \
    {                                                                                                                                                    \
      ddr->registerVariable< int >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbInt, boost::ref( name ), #name, _1 ), desc, min, max );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                            \
    }

  #define dynreconfRegIntGroup( name, desc, min, max, group )                                                                                                  \
    {                                                                                                                                                           \
      ddr->registerVariable< int >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbInt, boost::ref( name ), #name, _1 ), desc, min, max, group );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                   \
    }

  #define dynreconfRegDouble( name, desc, min, max )                                                                                                          \
    {                                                                                                                                                          \
      ddr->registerVariable< double >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbDouble, boost::ref( name ), #name, _1 ), desc, min, max );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                  \
    }

  #define dynreconfRegDoubleGroup( name, desc, min, max, group )                                                                                                     \
    {                                                                                                                                                                 \
      ddr->registerVariable< double >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbDouble, boost::ref( name ), #name, _1 ), desc, min, max, group );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                         \
    }

  #define dynreconfRegStr( name, desc )                                                                                                                \
    {                                                                                                                                                   \
      ddr->registerVariable< std::string >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbStr, boost::ref( name ), #name, _1 ), desc );    \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                           \
    }

  #define dynreconfRegStrGroup( name, desc, group )                                                                                                                   \
    {                                                                                                                                                                  \
      ddr->registerVariable< std::string >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbStr, boost::ref( name ), #name, _1 ), desc, "", "", group );    \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                          \
    }

  #define dynreconfRegBool( name, desc )                                                                                                         \
    {                                                                                                                                             \
      ddr->registerVariable< bool >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbBool, boost::ref( name ), #name, _1 ), desc );    \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                     \
    }

  #define dynreconfRegBoolGroup( name, desc, group )                                                                                                                 \
    {                                                                                                                                                                 \
      ddr->registerVariable< bool >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbBool, boost::ref( name ), #name, _1 ), desc, false, true, group );    \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                         \
    }

  #define dynreconfRegInt_boolOut( name, desc, min, max, bool_out )                                                                                                                \
    {                                                                                                                                                                               \
      ddr->registerVariable<int>( #name , name, boost::bind( fkie_ddynamic_reconfigure::paramCbInt_boolOut, boost::ref( name, #name , boost::ref(bool_out) , _1), desc, min, max);   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                                       \
    }

  #define dynreconfRegDouble_boolOut( name, desc, min, max, bool_out )                                                                                                                        \
    {                                                                                                                                                                                          \
      ddr->registerVariable< double >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbDouble_boolOut, boost::ref( name ), #name, boost::ref( bool_out ), _1 ), desc, min, max );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                                                  \
    }

  #define dynreconfRegStr_boolOut( name, desc, bool_out )                                                                                                                             \
    {                                                                                                                                                                                  \
      ddr->registerVariable< std::string >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbStr_boolOut, boost::ref( name ), #name, boost::ref( bool_out ), _1 ), desc );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                                          \
    }

  #define dynreconfRegBool_boolOut( name, desc, bool_out )                                                                                                                      \
    {                                                                                                                                                                            \
      ddr->registerVariable< bool >( #name, name, boost::bind( fkie_ddynamic_reconfigure::paramCbBool_boolOut, boost::ref( name ), #name, boost::ref( bool_out ), _1 ), desc );   \
      ROS_INFO_STREAM_NAMED( "GlobalParams", "param dyn '" << #name << "' : '" << name << "'");                                                                                    \
    }

  #define loadStaticParameter( variable )                                                             \
  {                                                                                                    \
    ros::NodeHandle( "~" ).param( #variable, variable, variable );                                      \
    ros::NodeHandle( "~" ).setParam( #variable, variable );                                              \
    ROS_INFO_STREAM_NAMED( "GlobalParams",  "param stc '" << #variable << "' : '" << variable << "'" );   \
  }

    void paramCbInt(int& var, std::string s, int new_value);
    void paramCbDouble(double& var, std::string s, double new_value);
    void paramCbStr(std::string& var, std::string s, std::string new_value);
    void paramCbBool(bool& var, std::string s, bool new_value);
    void paramCbInt_boolOut(int& var, std::string s, bool& bool_out, int new_value);
    void paramCbDouble_boolOut(double& var, std::string s, bool& bool_out, double new_value);
    void paramCbStr_boolOut(std::string& var, std::string s, bool& bool_out, std::string new_value);
    void paramCbBool_boolOut(bool& var, std::string s, bool& bool_out, bool new_value);


  template< typename T >
  class GlobalParameters
  {
  protected:
    boost::thread boost_thread;
    static T* globalPtr;
    void dynReconfSetParam(dynamic_reconfigure::Config& conf);
    std::unique_ptr< fkie_ddynamic_reconfigure::DDynamicReconfigure > ddr;

  public:

    GlobalParameters()
    {
      ddr = std::make_unique< fkie_ddynamic_reconfigure::DDynamicReconfigure >( ros::NodeHandle ( "~" ) );
    }

    static T& getInstance()
    {
      if (globalPtr == nullptr)
        globalPtr = new T;
      return *globalPtr;
    }

    fkie_ddynamic_reconfigure::DDynamicReconfigure* getDdrPtr() const
    {
      return ddr.get();
    }

  };
  template <class T> T* GlobalParameters<T>::globalPtr = NULL;
}

#endif /* VIRTUALGLOBALPARAMETERS_H_ */
