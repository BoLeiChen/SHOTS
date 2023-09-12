#!/usr/bin/env python
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

import random
from builtins import *
from builtins import object, range
from typing import (Dict, List)

import cv2 as cv
import numpy as np
import rospy
from fkie_measurement_server import Singleton
from fkie_measurement_server.modules.InterpolationModule import InterpolationModule
from fkie_measurement_server.MeasurementServer import MeasurementServer
from fkie_measurement_server.modules.SourceLocalizationModule import SourceLocalizationModule
from fkie_measurement_server.ParameterManager import ParameterManager
from future import standard_library
from future.utils import with_metaclass
from geometry_msgs.msg import Point
from past.utils import old_div
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path

standard_library.install_aliases()


class BoundaryPolygonModule(with_metaclass(Singleton, object)):
    def __init__(self):
        self.initialized = False
        self.params = ParameterManager()
        self.m_interpolator = InterpolationModule()
        self.m_server = MeasurementServer()
        self.m_source_location = SourceLocalizationModule()

        self.polygons = {}  # type: Dict[str, List[List[List[float]]]]
        self.max_heights = {}  # type: Dict[str, float]

        if len(self.params.bp_threshold_list) == 0:
            rospy.logwarn(
                "[BoundaryPolygonModule] threshold_list is empty [~boundary_polygons/threshold_list], using [~boundary_polygons/threshold_percentage]")

        self.typed_threshold = {}
        for t in self.params.bp_threshold_list:
            self.typed_threshold[list(t.items())[0][0]] = list(t.items())[0][1]

        self.color_lines = ColorRGBA()
        self.color_lines.r = 0.35
        self.color_lines.g = 0.34
        self.color_lines.b = 0.38
        self.color_lines.a = self.params.bp_marker_alpha

        self.pub_boundaries = rospy.Publisher(
            "boundary_polygons", MarkerArray, queue_size=5, latch=True)

        self.clear_markers()

        # subscribe for updates on manual boundary
        if self.params.bp_use_manual_box:
            self.current_boundary_msg = None
            self.sub_manual_boundary = rospy.Subscriber(
                'manual_boundary', Path, self.callback_boundary, queue_size=1)

        self.initialized = True  # type: bool

    def compute_boundary_polygons(self):
        # type: (...) -> None
        """
        Computes a list of polygons (2D) based on local maxima points
        """
        if not self.initialized:
            rospy.loginfo(
                "[BoundaryPolygonModule] Module not initialized correctly")
            return

        if self.params.bp_use_manual_box:
            self.compute_manual_boundary()
        elif self.params.bp_use_rectangular_box:
            self.compute_bounding_box()
        else:
            self.compute_measurement_polygons()

    def callback_boundary(self, msg):
        self.current_boundary_msg = msg
        rospy.loginfo(
            "[BoundaryPolygonModule] New perimeter received with [" + str(len(msg.poses)) + "] poses")

    def compute_manual_boundary(self):
        rospy.loginfo("computing polygons based on manual boundary")

        # get source locations
        for (m_type, _) in list(self.m_source_location.source_locations.items()):
            self.polygons[m_type] = []
            self.max_heights[m_type] = np.max(
                self.m_interpolator.lin_space_grid[m_type][2])

            # create bounding box based on [bp_rectangular_box_size]
            con_xy = []  # type: List[List[float]]

            if self.current_boundary_msg is not None:
                for pose in self.current_boundary_msg.poses:
                    con_xy.append([pose.pose.position.x, pose.pose.position.y])

            self.polygons[m_type].append(con_xy)

    def compute_bounding_box(self):
        rospy.loginfo("computing polygons based on bounding box")

        # get source locations
        for (m_type, locations) in list(self.m_source_location.source_locations.items()):
            self.polygons[m_type] = []
            self.max_heights[m_type] = np.max(
                self.m_interpolator.lin_space_grid[m_type][2])

            for location in locations:
                l_x = location[0]
                l_y = location[1]
                dx = self.params.bp_rectangular_box_size_x / 2.0
                dy = self.params.bp_rectangular_box_size_y / 2.0

                # create bounding box based on [bp_rectangular_box_size]
                con_xy = []  # type: List[List[float]]

                con_xy.append([l_x - dx, l_y - dy])
                con_xy.append([l_x - dx, l_y + dy])
                con_xy.append([l_x + dx, l_y + dy])
                con_xy.append([l_x + dx, l_y - dy])

                self.polygons[m_type].append(con_xy)

    def compute_measurement_polygons(self):
        rospy.loginfo("computing polygons based on measurements")

        for (m_type, max_grid) in list(self.m_interpolator.max_mesh_grid.items()):
            grid_2d = np.zeros((len(self.m_interpolator.lin_space_grid[m_type][0]),
                                len(self.m_interpolator.lin_space_grid[m_type][1])), dtype=np.uint16)

            index_max_grid = 0
            for (xi, _) in enumerate(self.m_interpolator.lin_space_grid[m_type][0]):
                for (yi, _) in enumerate(self.m_interpolator.lin_space_grid[m_type][1]):
                    grid_2d[xi][yi] = int(
                        max(0.0, max_grid[index_max_grid][2]))
                    index_max_grid += 1

            max_value = self.m_server.data[m_type].get_max_value()
            if max_value < 0.001:
                rospy.logwarn(
                    "compute_measurement_polygons: max value too low")
                continue

            #blur_img = cv.blur(grid_2d.astype(np.uint16), (4, 4))
            #_, thresh_grid = cv.threshold(blur_img, max_value * self.threshold, 255, 0)
            #thresh_grid = cv.adaptiveThreshold(grid_2d.astype(np.uint8), min(255, max_value * self.threshold), cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 9, 0)

            # thresholding
            threshold_value = max_value * self.params.bp_threshold_percentage

            if m_type in self.typed_threshold:
                threshold_value = self.typed_threshold[m_type]
                rospy.loginfo("Threshold value for {0}: {1}".format(
                    m_type, threshold_value))

            #max_sorted = np.sort(max_grid_T[3])
            #threshold_value = max_sorted[-self.threshold]

            _, thresh_grid = cv.threshold(grid_2d.astype(
                np.uint16), int(threshold_value), 255, 0)
            #thresh_grid = cv.adaptiveThreshold(grid_2d.astype(np.uint8), threshold_value, cv.ADAPTIVE_THRESH_GAUSSIAN_C, cv.THRESH_BINARY, 9, 0)

            self.polygons[m_type] = []
            contours, _ = cv.findContours(thresh_grid.astype(
                np.uint8), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
            self.max_heights[m_type] = np.max(
                self.m_interpolator.lin_space_grid[m_type][2])

            for contour in contours:
                con_xy = []  # type: List[List[float]]
                for p in contour:
                    xi = p[0][1]  # keep indices inverted
                    yi = p[0][0]
                    con_xy.append([self.m_interpolator.lin_space_grid[m_type][0][xi],
                                   self.m_interpolator.lin_space_grid[m_type][1][yi]])

                self.polygons[m_type].append(con_xy)

    def publish_boundary_polygons(self):
        # type: (...) -> None
        """
        Publish [MarkerArray] with computed boundaries for visualization in RVIZ
        """
        if not self.initialized:
            rospy.loginfo(
                "[BoundaryPolygonModule] Module not initialized correctly")
            return

        if self.pub_boundaries.get_num_connections() == 0:
            rospy.logwarn(
                "BoundaryPolygonModule: Skipping boundary publishing, because no subscribers have been found.")
            return

        if len(self.polygons) == 0 or len(self.max_heights) == 0:
            rospy.logwarn("BoundaryPolygonModule: Invalid boundary polygon")
            return

        self.clear_markers()

        # create markers
        marker_polygon_array = MarkerArray()

        for (m_type, contours) in list(self.polygons.items()):
            m = Marker()
            m.header.stamp = rospy.Time()
            m.header.frame_id = self.params.global_frame
            m.ns = m_type + "_boundaries"
            m.id = 0
            m.action = Marker.ADD
            m.type = Marker.TRIANGLE_LIST
            m.pose.orientation.w = 1.0
            m.scale.x = 1.0
            m.scale.y = 1.0
            m.scale.z = 1.0
            m.color.a = self.params.bp_marker_alpha

            counter_id = 0
            # generated_colors = []

            if len(contours) == 0:
                rospy.loginfo("publish_boundary_polygons: Empty [contours]")
                continue

            max_height = self.max_heights[m_type] if self.params.bp_max_height <= 0.0 else self.params.bp_max_height

            for (i_contour, contour) in enumerate(contours):
                # skip empty contours
                if len(contour) == 0 or len(contour[0]) == 0:
                    continue

                p_color = ColorRGBA()
                c = self.generate_random_pastel_color(pastel_factor=0.9)

                # p_color.r = c[0]
                # p_color.g = c[1]
                # p_color.b = c[2]
                # p_color.a = self.params.bp_marker_alpha

                p_color.r = 1.0
                p_color.g = 0.40
                p_color.b = 0.40
                p_color.a = self.params.bp_marker_alpha

                # add marker Text
                m_text = Marker()
                m_text.header.stamp = rospy.Time()
                m_text.header.frame_id = self.params.global_frame
                m_text.ns = m_type + "_description"
                m_text.id = i_contour
                m_text.action = Marker.ADD
                m_text.type = Marker.TEXT_VIEW_FACING
                m_text.pose.position.x = contour[0][0]
                m_text.pose.position.y = contour[0][1]
                m_text.pose.position.z = max_height + 0.7
                m_text.pose.orientation.w = 1.0
                m_text.scale.z = 0.5
                m_text.color = self.color_lines
                m_text.text = "{0} - ID: {1}".format(m_type, i_contour)
                marker_polygon_array.markers.append(m_text)

                for i in range(0, len(contour)):
                    c1 = contour[i-1]
                    c2 = contour[i]

                    p1 = Point()
                    p1.x = c1[0]
                    p1.y = c1[1]
                    p1.z = self.params.bp_min_height

                    p2 = Point()
                    p2.x = c1[0]
                    p2.y = c1[1]
                    p2.z = max_height

                    p3 = Point()
                    p3.x = c2[0]
                    p3.y = c2[1]
                    p3.z = self.params.bp_min_height

                    p4 = Point()
                    p4.x = c2[0]
                    p4.y = c2[1]
                    p4.z = max_height

                    # first triangle
                    m.points.append(p1)
                    m.points.append(p2)
                    m.points.append(p3)

                    # second triangle
                    m.points.append(p3)
                    m.points.append(p4)
                    m.points.append(p2)

                    for _ in range(6):
                        m.colors.append(p_color)

                    # add marker arrow
                    m_lines = Marker()
                    m_lines.header.stamp = rospy.Time()
                    m_lines.header.frame_id = self.params.global_frame
                    m_lines.ns = m_type + "_boundaries_line"
                    m_lines.id = counter_id
                    m_lines.action = Marker.ADD
                    m_lines.type = Marker.ARROW
                    m_lines.pose.orientation.w = 1.0
                    m_lines.frame_locked = True
                    m_lines.scale.x = 0.05
                    m_lines.scale.y = 0.05
                    m_lines.scale.z = 0.001
                    m_lines.color = self.color_lines

                    # create lines
                    m_lines.points.append(p3)
                    m_lines.points.append(p4)

                    marker_polygon_array.markers.append(m_lines)

                    counter_id += 1

            if len(m.points) > 0:
                marker_polygon_array.markers.append(m)
            else:
                rospy.logwarn("No points added for marker: " + m_type)

        if len(marker_polygon_array.markers) > 0:
            self.pub_boundaries.publish(marker_polygon_array)
        else:
            rospy.logwarn("No boundary polygons were added to the marker")

    def clear_markers(self):
        # type: (...) -> None
        """
        Send a DELETEALL marker to clear visualization
        """
        if not self.initialized:
            rospy.loginfo(
                "[BoundaryPolygonModule] Module not initialized correctly")
            return

        ma = MarkerArray()
        m = Marker()
        m.header.stamp = rospy.Time()
        m.header.frame_id = self.params.global_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.pub_boundaries.publish(ma)
        rospy.loginfo("[BoundaryPolygonModule] Clearing markers")

    def generate_random_pastel_color(self, pastel_factor=0.5):
        # type: (float) -> List[float]
        """
        Generate random colors using [pastel_factor] 
        """
        return [old_div((x+pastel_factor), (1.0+pastel_factor)) for x in [random.uniform(0, 1.0) for _ in [1, 2, 3]]]
