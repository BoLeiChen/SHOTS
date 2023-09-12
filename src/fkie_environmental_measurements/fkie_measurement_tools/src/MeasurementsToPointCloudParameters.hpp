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

#ifndef MEASUREMENTS_TO_POINT_CLOUD_PARAMETERS_H_
#define MEASUREMENTS_TO_POINT_CLOUD_PARAMETERS_H_

#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

class MeasurementsToPointCloudParameters
  : public fkie_ddynamic_reconfigure::GlobalParameters<MeasurementsToPointCloudParameters>
{
protected:
  static MeasurementsToPointCloudParameters* globalPtr;

public:
  std::string topic_request_interpolation = "request_interpolation";
  std::string source_type = "DoseRate";

  double spin_rate = 1.0;
  double minimum_variance = 0.08;
  double minimum_mean = 0.0;
  double initial_max_mean_value = 0.0;

  MeasurementsToPointCloudParameters()
  {
    loadStaticParameter(topic_request_interpolation);
    loadStaticParameter(source_type);

    dynreconfRegDoubleGroup(spin_rate, "spin rate", 1.0, 10.0, "Interpolation");

    dynreconfRegDoubleGroup(minimum_variance, "Minumum variance to consider a valid estimation", 0.0, 10.0,
                            "Interpolation");
    dynreconfRegDoubleGroup(minimum_mean, "Minumum mean value", 0.0, 500.0, "Interpolation");
    dynreconfRegDoubleGroup(initial_max_mean_value, "Initial maximum mean value", 0.0, 20.0, "Interpolation");

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

#endif /* MEASUREMENTS_TO_POINT_CLOUD_PARAMETERS_H_ */
