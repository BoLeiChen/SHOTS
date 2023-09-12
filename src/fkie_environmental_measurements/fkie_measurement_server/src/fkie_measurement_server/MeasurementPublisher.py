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
from builtins import object, range

import rospy
import tf2_ros
from fkie_measurement_server import Singleton
from fkie_measurement_server.MeasurementServer import MeasurementServer
from fkie_measurement_server.ParameterManager import ParameterManager
from fkie_measurement_server.utils import *
from future import standard_library
from future.utils import with_metaclass
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from fkie_measurement_msgs.msg import MeasurementArray, MeasurementLocated, MeasurementValue

standard_library.install_aliases()


class MeasurementPublisher(with_metaclass(Singleton, object)):
    def __init__(self):
        self.params = ParameterManager()
        self.ms = MeasurementServer()
        self.pub_measurement_markers = rospy.Publisher(
            "measurement_marker", MarkerArray, queue_size=5)
        self.pub_measurement_array = rospy.Publisher(
            "measurement_array", MeasurementArray, queue_size=5)
        self.clear_markers()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def clear_markers(self):
        # type: (...) -> None

        # clear measurement marker
        ma = MarkerArray()
        m = Marker()
        m.id = 0
        m.header.frame_id = self.params.global_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.pub_measurement_markers.publish(ma)

    def publish_measurements(self):
        # type: (...) -> None

        publish_measurement_markers = True
        publish_measurement_array = True

        if self.pub_measurement_markers.get_num_connections() == 0:
            rospy.logdebug(
                "No subscribers are registered, skipping the publication of measurement markers")
            publish_measurement_markers = False

        if self.pub_measurement_array.get_num_connections() == 0:
            rospy.logdebug(
                "No subscribers are registered, skipping the publication of measurement array")
            publish_measurement_array = False

        # create markers
        if publish_measurement_markers:
            markers_to_publish = MarkerArray()
        # create measurement array
        if publish_measurement_array:
            measurement_array = MeasurementArray()
            measurement_array.header.stamp = rospy.Time.now()
            measurement_array.full_history = True

        with self.ms.ms_mutex:
            for m_type, m_data in list(self.ms.data.items()):
                # create point marker
                if publish_measurement_markers:
                    m = Marker()
                    m.header.stamp = rospy.Time()
                    m.header.frame_id = self.params.global_frame
                    m.ns = m_type + "_data"
                    m.id = 0
                    m.action = Marker.ADD
                    m.type = Marker.SPHERE_LIST
                    m.pose.orientation.w = 1.0
                    m.scale.x = self.params.mp_marker_size
                    m.scale.y = self.params.mp_marker_size
                    m.scale.z = self.params.mp_marker_size
                    m.color.a = 0.8

                with m_data.lookup_mutex:
                    for key, val in list(m_data.positions.items()):
                        stamp = m_data.positions[key][0]
                        value = m_data.positions[key][1]

                        # add point data to marker
                        if publish_measurement_markers:
                            pp = Point()
                            pp.x = key[0]
                            pp.y = key[1]
                            pp.z = key[2]
                            m.points.append(pp)
                            if isinstance(value, float):
                                c = compute_color(value, m_data.get_min_value(),
                                                  m_data.get_max_value())
                                if self.params.mp_marker_alpha >= 0.0:
                                    c.a = self.params.mp_marker_alpha

                                m.colors.append(c)

                        if publish_measurement_array:
                            # create measurement value message
                            measurement_value = MeasurementValue()
                            measurement_value.begin = stamp
                            measurement_value.end = stamp
                            measurement_value.sensor = m_type
                            measurement_value.source_type = m_type
                            measurement_value.unit = m_data.unit
                            measurement_value.value_text = value if isinstance(
                                value, str) else ''
                            measurement_value.value_single = value if isinstance(
                                value, float) else 0.0
                            measurement_value.value_array = value if isinstance(
                                value, tuple) else []

                            # check existent measurement located (in array) for current position
                            measurement_located = next((m_located for m_located in measurement_array.located_measurements
                                                        if m_located.pose.pose.position.x == key[0] and
                                                        m_located.pose.pose.position.y == key[1] and
                                                        m_located.pose.pose.position.z == key[2]), None)

                            # add sensor data if measurement located was found
                            if measurement_located:
                                measurement_located.measurement.values.append(
                                    measurement_value)

                            # else create a new one
                            else:
                                measurement_located = MeasurementLocated()
                                measurement_located.pose.header.frame_id = m_data.frame_id
                                measurement_located.pose.header.stamp = stamp
                                measurement_located.pose.pose.position.x = key[0]
                                measurement_located.pose.pose.position.y = key[1]
                                measurement_located.pose.pose.position.z = key[2]
                                measurement_located.measurement.header.stamp = stamp

                                measurement_located.measurement.values.append(
                                    measurement_value)
                                measurement_array.located_measurements.append(
                                    measurement_located)

                if publish_measurement_markers:
                    # # Publish grid-fixed measurements
                    # values = m_data.grid.get_values()  # List[x, y, z, value]
                    # for i_v in range(len(values)):
                    #     pp = Point()
                    #     pp.x = values[i_v][0]
                    #     pp.y = values[i_v][1]
                    #     pp.z = values[i_v][2]

                    #     value = values[i_v][3]
                    #     c = compute_color(value, m_data.get_min_value(), m_data.get_max_value())
                    #     c.a = self.params.mp_marker_alpha

                    #     m.points.append(pp)
                    #     m.colors.append(c)

                    markers_to_publish.markers.append(m)

        if publish_measurement_markers:
            self.pub_measurement_markers.publish(markers_to_publish)

        if publish_measurement_array:
            # sort measurement array in ascending stamp order
            measurement_array.located_measurements.sort(
                key=lambda x: x.measurement.header.stamp.to_sec())
            # publish measurement array
            self.pub_measurement_array.publish(measurement_array)
