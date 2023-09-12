# fkie_ddynamic_reconfigure

The fkie_ddynamic_reconfigure package is fixed and extended version of [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) that allows modifying parameters of a ROS node using the dynamic_reconfigure framework without having to write cfg files. Further, dynamic creation of dynamic parameters is possible. Dynamic dynamic parameters!

To simplify the usage, a header file "GlobalParameters.h" is included. It supports fast creation of static and dynamic parameters and a sigleton mechanism. Currently it supports bools, ints, doubles and strings. There also is an usage example in "test/MyGlobalParameters.h".

## Usage of GlobalParameters.h

To take advantage of the sigleton mechanism, the parameter class has to be initialized as a pointer, e.g.:

```cpp
MyGlobalParameters* params = &MyGlobalParameters::getInstance();
```
If no params object exists, a new one will be created. If there already is one, the existing one is used. Singleton.

As in "MyGlobalParameters.h" the GlobalParameters class has to be inherited. In the new class variables for the parameters are added and the constructor is overwritten with parameter loading as follows: 

```cpp
#ifndef MY_GLOBALPARAMETERS_H_
#define MY_GLOBALPARAMETERS_H_

#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

class MyGlobalParameters : public fkie_ddynamic_reconfigure::GlobalParameters<MyGlobalParameters>
{
  protected:
    static MyGlobalParameters* globalPtr;

  public:

    double hz = 2.0;
    std::string frame_robot = "base_link";
    std::string topic_cmd_pos = "cmd_pos";
    double distance = 10.0;
    bool enabled = false;

    MyGlobalParameters()
    {
      loadStaticParameter( hz );
      loadStaticParameter( frame_robot );
      loadStaticParameter( topic_cmd_pos );

      dynreconfRegDouble( distance, "distance to target frame", 0.0, 100.0 );
      dynreconfRegBool( enabled, "on/off" );

      ddr->publishServicesTopics();
    }

};

MyGlobalParameters* MyGlobalParameters::globalPtr = 0;

#endif /* MYGLOBALPARAMETERS_H_ */
```

```
```

```
```

```
```

```
```

## OLD Usage (ddynamic_reconfigure)

Modifying in place a variable:
```cpp
#include <ros/ros.h>
#include <fkie_ddynamic_reconfigure/fkie_ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    fkie_ddynamic_reconfigure::DDynamicReconfigure ddr;
    int int_param = 0;
    ddr.registerVariable<int>("int_param", &int_param, "param description");
    ddr.publishServicesTopics();
    // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the variable int_param is updated automatically
    
    int_param = 10; //This will also update the dynamic_reconfigure tools with the new value 10
    ros::spin();
    return 0;
 }
```

Modifying a variable via a callback:
```cpp
#include <ros/ros.h>
#include <fkie_ddynamic_reconfigure/fkie_ddynamic_reconfigure.h>

int global_int;

void paramCb(int new_value)
{
   global_int = new_value;
   ROS_INFO("Param modified");
}

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    fkie_ddynamic_reconfigure::DDynamicReconfigure ddr;
    
    ddr.registerVariable<int>("int_param", 10 /* initial value */, boost::bind(paramCb, _1), "param description");
    ddr.publishServicesTopics();
    // Now parameter can be modified from the dynamic_reconfigure GUI or other tools and the callback is called on each update
    ros::spin();
    return 0;
 }
```

Registering an enum:

```cpp

#include <ros/ros.h>
#include <fkie_ddynamic_reconfigure/fkie_ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh;
    fkie_ddynamic_reconfigure::DDynamicReconfigure ddr;
    
    std::map<std::string, std::string> enum_map = {{"Key 1", "Value 1"}, {"Key 2", "Value 2"}};
    std::string enum_value = enum_map["Key 1"];
    ddr.registerEnumVariable<std::string>("string_enum", &enum_value,"param description", enum_map);
    ddr.publishServicesTopics();
    ros::spin();
    return 0;
 }
```

Registering variables in a private namespace "ddynamic_tutorials/other_namespace/int_param":

```cpp

#include <ros/ros.h>
#include <fkie_ddynamic_reconfigure/fkie_ddynamic_reconfigure.h>

int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh("~/other_namespace");
    fkie_ddynamic_reconfigure::DDynamicReconfigure ddr(nh);

    int int_param = 0;
    ddr.registerVariable<int>("int_param", &int_param, "param description");
    ddr.publishServicesTopics();
    
    ros::spin();
    return 0;
}
```


Same scenario, but with the NodeHandle created after the ddr instantiation:

```cpp

#include <ros/ros.h>
#include <fkie_ddynamic_reconfigure/fkie_ddynamic_reconfigure.h>

std::unique_ptr<fkie_ddynamic_reconfigure::DDynamicReconfigure> ddr;
int main(int argc, char **argv) {
    // ROS init stage
    ros::init(argc, argv, "ddynamic_tutorials");
    ros::NodeHandle nh("~/other_namespace");

    ddr.reset(new fkie_ddynamic_reconfigure::DDynamicReconfigure(nh));
    int int_param = 0;
    ddr->registerVariable<int>("int_param", &int_param, "param description");
    ddr->publishServicesTopics();
    
    ros::spin();
    return 0;
}
```


## Issues
### Undefined reference to registerVariable or registerEnumVariable

These methods are templated, but the implementation is hidden, and there are explicit template instantiations for `int`, `bool`, `double` and `std::string`. If you are getting an undefined reference to one of these methods, make sure that you are passing parameters of this type.

## Third-party licences

The original work [ddynamic_reconfigure](https://github.com/pal-robotics/ddynamic_reconfigure) was released under BSD license.

## License

Our additional work is released under Apache 2.0 license.

```
Copyright 2022 Fraunhofer FKIE - All Rights Reserved

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```
