#!/usr/bin/env python3
# coding: utf8

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
from typing import (Any, Dict, List)

import rospy
from fkie_measurement_server import Singleton
from future import standard_library
from future.utils import with_metaclass

standard_library.install_aliases()


class SensorProperty(object):
    def __init__(self, unique_serial_id, frame_id=None, topic=None, topic_located=None):
        self.frame_id = frame_id
        self.unique_serial_id = unique_serial_id
        self.topic = topic
        self.topic_located = topic_located


class ParameterManager(with_metaclass(Singleton, object)):
    def __init__(self):
        # Measurement Exploration node params:
        self.rate = self.read_param('~rate', 2.0)  # type: float
        self.global_frame = self.read_param(
            '~global_frame', 'odom')  # type: str
        self.sensors = []  # type: List[SensorProperty]
        self.read_sensor_config_from_params()

        # Measurement Server Parameters
        m_name = '~measurement_server'
        self.measurement_xml_file = self.read_param(
            m_name + '/measurement_xml_file', 'measurements.xml')  # type: str
        self.ms_use_slam = self.read_param(
            m_name + '/use_slam', False)  # type: bool
        self.ms_grid_size = self.read_param(
            m_name + '/grid_size', 0.3)  # type: float
        self.ms_offset_x = self.read_param(
            m_name + '/offset_x', 0.0)  # type: float
        self.ms_offset_y = self.read_param(
            m_name + '/offset_y', 0.0)  # type: float
        self.ms_offset_z = self.read_param(
            m_name + '/offset_z', 0.0)  # type: float

        # Interpolation Module
        m_name = '~interpolation'
        self.it_grid_size = self.read_param(
            m_name + '/grid_size', 0.5)  # type: float
        self.it_marker_alpha = self.read_param(
            m_name + '/marker_alpha', 0.5)  # type: float
        self.it_min_alpha = self.read_param(
            m_name + '/min_alpha', 0.1)  # type: float
        self.it_min_mean_value = self.read_param(
            m_name + '/min_mean_value', 0.0)  # type: float
        self.it_padding = self.read_param(
            m_name + '/padding', 1.0)  # type: float

        self.it_method = self.read_param(
            m_name + '/method', 'RBF')  # type: str
        self.it_RBF_function = self.read_param(
            m_name + '/RBF/function', 'thin_plate')  # type: str
        self.it_LinearNDInterpolator_rescale = self.read_param(
            m_name + '/LinearNDInterpolator/rescale', False)  # type: bool
        self.it_NearestNDInterpolator_rescale = self.read_param(
            m_name + '/NearestNDInterpolator/rescale', False)  # type: bool
        # kriging_type: ordinary - universal
        self.it_OrdinaryKriging3D_kriging_type = self.read_param(
            m_name + '/OrdinaryKriging3D/kriging_type', 'ordinary')  # type: str
        # variogram_model: linear, power, spherical, gaussian, exponential
        self.it_OrdinaryKriging3D_variogram_model = self.read_param(
            m_name + '/OrdinaryKriging3D/variogram_model', 'gaussian')  # type: str

        # Boundary Polygon Module
        m_name = '~boundary_polygons'
        self.bp_marker_alpha = self.read_param(
            m_name + '/marker_alpha', 0.3)  # type: float
        self.bp_min_height = self.read_param(
            m_name + '/min_height', 0.0)  # type: float
        self.bp_max_height = self.read_param(
            m_name + '/max_height', 0.0)  # type: float
        self.bp_threshold_list = self.read_param(
            m_name + '/threshold_list', [])  # type: List[Dict[str, float]]
        self.bp_threshold_percentage = self.read_param(
            m_name + '/threshold_percentage', 0.7)  # type: float
        self.bp_use_manual_box = self.read_param(
            m_name + '/use_manual_box', False)  # type: bool
        self.bp_use_rectangular_box = self.read_param(
            m_name + '/use_rectangular_box', False)  # type: bool
        self.bp_rectangular_box_size_x = self.read_param(
            m_name + '/rectangular_box_size_x', 6.0)  # type: float
        self.bp_rectangular_box_size_y = self.read_param(
            m_name + '/rectangular_box_size_y', 6.0)  # type: float

        # Source localization
        m_name = '~source_localization'
        self.sl_marker_alpha = self.read_param(
            m_name + '/marker_alpha', 0.3)  # type: float

        # publisher module
        m_name = '~measurement_publisher'
        self.mp_marker_alpha = self.read_param(
            m_name + '/marker_alpha', 0.8)  # type: float
        self.mp_marker_size = self.read_param(
            m_name + '/marker_size', 0.2)  # type: float

        # SLAM params
        m_name = '~slam'
        self.slam_service_trajectory_query = self.read_param(
            m_name + '/service_trajectory_query', 'trajectory_query')  # type: str
        self.slam_current_trajectory_id = self.read_param(
            m_name + '/current_trajectory_id', 0)  # type: int
        self.slam_matching_dist_threshold = self.read_param(
            m_name + '/matching_dist_threshold', 0.1)  # type: float
        self.slam_trajectory_lookup_update_interval = self.read_param(
            m_name + '/trajectory_lookup_update_interval', 1.0)  # type: float
        self.slam_measurement_position_update_interval = self.read_param(
            m_name + '/measurement_position_update_interval', 2.0)  # type: float
        self.slam_robot_frame = self.read_param(
            m_name + '/robot_frame', 'base_link')  # type: str

    def read_sensor_config_from_params(self):
        # type: (...) -> None

        counter = 0
        param_name = "~measurement_server/sensors/sensor_{0}".format(counter)

        while rospy.has_param(param_name):
            unique_serial_id = rospy.get_param(
                "{0}/unique_serial_id".format(param_name), None)
            frame_id = rospy.get_param("{0}/frame_id".format(param_name), None)
            topic = rospy.get_param("{0}/topic".format(param_name), None)
            topic_located = rospy.get_param(
                "{0}/topic_located".format(param_name), None)

            self.sensors.append(SensorProperty(
                unique_serial_id, frame_id, topic, topic_located))
            rospy.loginfo("Adding sensor: unique_serial_id: {0}, frame_id: {1}, topic: {2}, topic_located: {3}".format(
                unique_serial_id, frame_id, topic, topic_located))

            counter += 1
            param_name = "~measurement_server/sensors/sensor_{0}".format(
                counter)

    # @overload
    # def read_param(self, param_name: str, default_value: int) -> int: ...

    # @overload
    # def read_param(self, param_name: str, default_value: float) -> float: ...

    # @overload
    # def read_param(self, param_name: str, default_value: str) -> str: ...

    # @overload
    # def read_param(self, param_name: str,
    #                default_value: List[Dict[str, float]]) -> List[Dict[str, float]]: ...

    def read_param(self, param_name, default_value):
        # type: (str, Any) -> Any
        param_value = rospy.get_param(param_name, default_value)
        rospy.loginfo("Parameter: [{0}]: [{1}]".format(
            param_name, param_value))
        return param_value
