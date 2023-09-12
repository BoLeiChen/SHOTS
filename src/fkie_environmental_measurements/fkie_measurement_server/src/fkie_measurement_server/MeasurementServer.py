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

from __future__ import (absolute_import, division, print_function,
                        unicode_literals)

import os.path
from builtins import *
from builtins import object, range
from typing import Dict, List

import numpy as np
import rospy
import tf2_ros
from fkie_measurement_server import Singleton
from fkie_measurement_server.ParameterManager import ParameterManager
from future import standard_library
from future.utils import with_metaclass
from fkie_measurement_msgs.msg import Measurement, MeasurementLocated
from threading import Lock

from fkie_measurement_server.classes.TypedMeasurement import TypedMeasurement

standard_library.install_aliases()

try:
    import xml.etree.cElementTree as ET
except ImportError:
    import xml.etree.ElementTree as ET


class MeasurementServer(with_metaclass(Singleton, object)):
    def __init__(self):
        self.params = ParameterManager()
        self.ms_mutex = Lock()

        self.data = {}  # type: Dict[str, TypedMeasurement]
        self.data_lock = Lock()

        self.measurement_types = []  # type: List[str]

        if len(self.params.sensors) == 0:
            rospy.logerr(
                "[MeasurementServer] Could not find/update sensor configuration")
            return

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # create subscribers for all registered sensors
        self.sub_sensor_list = []
        for s in self.params.sensors:
            if s.topic:
                sub_m = rospy.Subscriber(
                    s.topic, Measurement, self.callback_measurement, queue_size=5)
                self.sub_sensor_list.append(sub_m)

            if s.topic_located:
                sub_m_located = rospy.Subscriber(
                    s.topic_located, MeasurementLocated, self.callback_measurement_located, queue_size=5)
                self.sub_sensor_list.append(sub_m_located)

    def __str__(self):
        # type: (...) -> str
        return "Registered types: {0}, global_frame: {1}.".format(self.measurement_types, self.params.global_frame)

    def save_measurements(self):
        # type: (...) -> None
        self.export_to_xml(self.params.measurement_xml_file)

    def load_measurements(self):
        # type: (...) -> bool
        if not os.path.isfile(self.params.measurement_xml_file):
            rospy.logwarn(
                "File does not exists: {0}".format(self.params.measurement_xml_file))
            return False

        return self.import_from_xml(self.params.measurement_xml_file)

    def clear_server(self):
        # type: (...) -> None
        self.data_lock.acquire()
        try:
            self.data = {}
            self.measurement_types = []
        finally:
            self.data_lock.release()

    def add_measurement(self, source_type, unit, stamp, frame_id, position, value):
        # type: (str, str, int, str, List[float], float) -> bool
        if source_type != None and not np.isnan(position).any():
            self.data_lock.acquire()
            try:
                if source_type not in self.data:
                    self.data[source_type] = TypedMeasurement(
                        source_type, unit, frame_id, self.params.ms_grid_size, use_slam=self.params.ms_use_slam)
                    self.measurement_types.append(source_type)

                with self.ms_mutex:
                    self.data[source_type].add_measurement(
                        position, stamp, value)
            finally:
                self.data_lock.release()

            return True

        rospy.logwarn("Invalid measurement ignored.")
        return False

    def callback_measurement(self, msg):
        # type: (Measurement) -> None
        if not msg.unique_serial_id:
            rospy.logerr("[callback_measurement] empty [unique_serial_id].")
            return

        sensor_frame_id = ""
        for s in self.params.sensors:
            if s.unique_serial_id == msg.unique_serial_id:
                sensor_frame_id = s.frame_id

        if not sensor_frame_id:
            rospy.logerr(
                "[callback_measurement] got new message for sensor [{0}], which does not have any configuration".format(msg.unique_serial_id))
            return

        try:
            trans = self.tf_buffer.lookup_transform(
                self.params.global_frame, sensor_frame_id, rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("[callback_measurement] Could not find TF2 lookup between frames [{0}] and [{1}]".format(
                self.params.global_frame, sensor_frame_id
            ))
            return

        # compute position + offset
        msg_position = (
            trans.transform.translation.x + self.params.ms_offset_x,
            trans.transform.translation.y + self.params.ms_offset_y,
            trans.transform.translation.z + self.params.ms_offset_z
        )

        for msg_value in msg.values:
            if not self.add_measurement(msg_value.source_type,
                                        msg_value.unit,
                                        msg_value.header.stamp,
                                        self.params.global_frame,
                                        msg_position,
                                        msg_value.value_text or msg_value.value_array or msg_value.value_single):
                rospy.logwarn(
                    "[callback_measurement] Could not add new measurement for sensor {msg.unique_serial_id}")

    def callback_measurement_located(self, msg):
        # type: (MeasurementLocated) -> None
        if not msg.measurement.unique_serial_id:
            rospy.logerr(
                "[callback_measurement_located] empty [unique_serial_id].")
            return

        # compute position + offset
        msg_position = (
            msg.pose.pose.position.x + self.params.ms_offset_x,
            msg.pose.pose.position.y + self.params.ms_offset_y,
            msg.pose.pose.position.z + self.params.ms_offset_z
        )

        for msg_value in msg.values:
            if not self.add_measurement(msg_value.source_type,
                                        msg_value.unit,
                                        msg_value.header.stamp,
                                        msg.pose.header.frame_id,
                                        msg_position,
                                        msg_value.value_text or msg_value.value_array or msg_value.value_single):
                rospy.logwarn(
                    "[callback_measurement_located] Could not add new measurement for sensor {msg.unique_serial_id}")

    def export_to_xml(self, file_name):
        # type: (str) -> None
        xml_root = ET.Element("measurements")

        self.data_lock.acquire()
        try:
            for m_type, m_data in list(self.data.items()):
                xml_m_type = ET.SubElement(
                    xml_root, m_type, {'unit': m_data.unit, 'frame_id': m_data.frame_id})

                for key, val in list(m_data.positions.items()):
                    x = key[0]
                    y = key[1]
                    z = key[2]
                    secs = m_data.positions[key][0].secs + \
                        m_data.positions[key][0].nsecs/1000000000
                    value = m_data.positions[key][1]

                    ET.SubElement(
                        xml_m_type, "m").text = "{0:.2f};{1:.2f};{2:.2f};{3:.3f};{4}".format(x, y, z, secs, value)
        finally:
            self.data_lock.release()

        tree = ET.ElementTree(xml_root)
        tree.write(file_name)

    def import_from_xml(self, file_name):
        # type: (str) -> bool
        xml_root = ET.ElementTree(file=file_name)
        root = xml_root.getroot()
        self.data_lock.acquire()

        try:
            for xml_m_type in root:
                source_type = xml_m_type.tag
                unit = xml_m_type.attrib["unit"]
                frame_id = xml_m_type.attrib["frame_id"]

                if source_type not in self.data:
                    self.data[source_type] = TypedMeasurement(
                        source_type, unit, frame_id, self.params.ms_grid_size)
                    self.measurement_types.append(source_type)

                for node in xml_m_type:
                    if node.text is None or len(node.text) == 0:
                        continue

                    value = None
                    msg_position = (None, None, None)
                    stamp = None

                    # try to get first data with time-stamp
                    array_items = np.char.split(node.text, sep=';')
                    n_items = array_items.tolist()

                    if len(n_items) == 5:
                        # compute position + offset
                        msg_position = (
                            float(n_items[0]) + self.params.ms_offset_x,
                            float(n_items[1]) + self.params.ms_offset_y,
                            float(n_items[2]) + self.params.ms_offset_z
                        )

                        # time stamp (index 3) and value (index 4) are available
                        stamp = rospy.Time.from_sec(float(n_items[3]))

                        if n_items[4][0] == '(' and n_items[4][-1] == ')':
                            value = tuple(float(item)
                                          for item in n_items[4][1:-1].split(','))
                        else:
                            try:
                                value = float(n_items[4])
                            except ValueError:
                                value = n_items[4]
                    else:
                        # For compatibility:
                        # try to get data with only value
                        array_items = np.char.split(node.text, sep=',')
                        n_items = array_items.tolist()

                        if len(n_items) == 4:
                            # compute position + offset
                            msg_position = (
                                float(n_items[0]) + self.params.ms_offset_x,
                                float(n_items[1]) + self.params.ms_offset_y,
                                float(n_items[2]) + self.params.ms_offset_z
                            )

                            # For compatibility: only value (index 3) is available
                            stamp = rospy.Time.now()
                            value = float(n_items[3])
                        else:
                            rospy.logwarn(
                                "[import_from_xml] invalid measurement: {0}".format(node.text))

                    # Add new measurement to server
                    if value is not None and stamp is not None:
                        self.data[source_type].add_measurement(
                            msg_position, stamp, value)

        finally:
            self.data_lock.release()

        return len(self.data) > 0
