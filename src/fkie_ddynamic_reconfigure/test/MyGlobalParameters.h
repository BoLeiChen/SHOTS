/*************************************************************************fkie
 * FRAUNHOFER INTERNAL                                                       *
 *****************************************************************************
 *                                                                           *
 * Copyright 2006-2021 Fraunhofer FKIE                                       *
 * All rights reserved                                                       *
 *                                                                           *
 * RIGHT OF USE. The contents of this file may neither be passed on to third *
 * parties nor utilized or divulged without the expressed prior permission   *
 * of Fraunhofer.                                                            *
 *                                                                           *
 * DEFENCE MATERIAL. This file may contain information which is subject to   *
 * export control.                                                           *
 *                                                                           *
 * FRAUNHOFER INTERNAL. The contents of this file are restricted to the use  *
 * at Fraunhofer, sponsoring German agencies, and other project partners in  *
 * accordance with contract clauses and/or written agreements.               *
 *                                                                           *
 *****************************************************************************/

/*
 * GlobalParameters.h
 *
 *  Created on: Mar 20, 2021
 *      Author: hoeller
 * 
 * example inheritance for GlobalParameters.h
 */

#ifndef MY_GLOBALPARAMETERS_H_
#define MY_GLOBALPARAMETERS_H_

#include <fkie_ddynamic_reconfigure/GlobalParameters.h>

class MyGlobalParameters : public fkie_ddynamic_reconfigure::GlobalParameters<MyGlobalParameters>
{

  public:

    double hz = 2.0;
    std::string frame_robot = "target";
    std::string frame_target = "base_link";
    std::string topic_cmd_pos = "cmd_pos";
    std::string topic_cmd_pos_passthrough = "crude_follower_passthrough";
    std::string topic_prefix_power_manual_override = "disabled";
    std::string power_manual_override_text = "force follow";
    double distance = 10.0;
    double threshold = 6.5;
    bool enabled = false;

    MyGlobalParameters()
    {
      loadStaticParameter( hz );
      loadStaticParameter( frame_robot );
      loadStaticParameter( frame_target );
      loadStaticParameter( topic_cmd_pos );
      loadStaticParameter( topic_cmd_pos_passthrough );
      loadStaticParameter( topic_prefix_power_manual_override );
      loadStaticParameter( power_manual_override_text );

      dynreconfRegDouble( distance, "distance to target frame", 0.0, 100.0 );
      dynreconfRegDouble( threshold, "distance threshold", 0.0, 100.0 );
      dynreconfRegBool( enabled, "on/off" );

      ddr->publishServicesTopics();
    }

};

#endif /* MYGLOBALPARAMETERS_H_ */
