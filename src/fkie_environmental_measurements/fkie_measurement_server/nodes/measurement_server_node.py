#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#  Copyright 2022 Fraunhofer FKIE - All Rights Reserved
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

from builtins import *
from builtins import object

import rospy

from fkie_measurement_server.MeasurementInterfaces import MeasurementInterfaces
from fkie_measurement_server.MeasurementPublisher import MeasurementPublisher
from fkie_measurement_server.MeasurementServer import MeasurementServer
from fkie_measurement_server.ParameterManager import ParameterManager

from fkie_measurement_server.modules.BoundaryPolygonModule import BoundaryPolygonModule
from fkie_measurement_server.modules.InterpolationModule import InterpolationModule
from fkie_measurement_server.modules.SourceLocalizationModule import SourceLocalizationModule

from future import standard_library
standard_library.install_aliases()


class MeasurementServerNode(object):
    def __init__(self):
        self.params = ParameterManager()

        # initialize singletone objects
        self.m_params = ParameterManager()
        self.m_measurement_server = MeasurementServer()
        self.m_publisher = MeasurementPublisher()
        self.m_interpolation = InterpolationModule()
        self.m_boundary_polygon = BoundaryPolygonModule()
        self.m_source_location = SourceLocalizationModule()
        self.m_interfaces = MeasurementInterfaces()

        # clear existing markers
        self.m_publisher.clear_markers()
        self.m_boundary_polygon.clear_markers()
        self.m_interpolation.clear_markers()

    def spin(self):
        # read measurements at start
        self.m_interfaces.request_load_measurements = True

        r = rospy.Rate(self.params.rate)
        while not rospy.is_shutdown():
            self.m_interfaces.spin()
            r.sleep()


# Main function
if __name__ == '__main__':
    rospy.init_node('MeasurementServerNode')
    mes = MeasurementServerNode()
    mes.spin()
