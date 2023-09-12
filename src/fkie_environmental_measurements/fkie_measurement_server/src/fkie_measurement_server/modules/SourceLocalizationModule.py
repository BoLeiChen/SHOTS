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
from builtins import object, range, zip

import numpy as np
import rospy
from fkie_measurement_server import Singleton
from fkie_measurement_server.ParameterManager import ParameterManager
from future import standard_library
from future.utils import with_metaclass
from geometry_msgs.msg import Point
from past.utils import old_div
from scipy import ndimage as ndi
from skimage.feature import peak_local_max
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

standard_library.install_aliases()


class SourceLocalizationModule(with_metaclass(Singleton, object)):
    def __init__(self):
        self.params = ParameterManager()

        self.source_locations = {}

        # ROS interface (publisher/subscribers)
        self.pub_source_loc = rospy.Publisher(
            "source_location", MarkerArray, queue_size=5)

    def compute_source_location(self, interpolator):
        rospy.loginfo("Computing source locations")

        for (m_type, max_grid) in list(interpolator.max_mesh_grid.items()):
            grid_2d = np.zeros((len(interpolator.lin_space_grid[m_type][0]),
                                len(interpolator.lin_space_grid[m_type][1])), dtype=np.uint16)

            index_max_grid = 0
            for (xi, _) in enumerate(interpolator.lin_space_grid[m_type][0]):
                for (yi, _) in enumerate(interpolator.lin_space_grid[m_type][1]):
                    grid_2d[xi][yi] = int(
                        max(0.0, max_grid[index_max_grid][2]))
                    index_max_grid += 1

            #max_grid_T = map(list, zip(*max_grid))
            #max_value = np.max(max_grid_T[3])

            image_max = ndi.maximum_filter(grid_2d, size=1, mode='reflect')

            # decrease [current_min_distance] until max point have be found
            current_min_distance = 6
            coordinates = []
            while len(coordinates) == 0 and current_min_distance > 0:
                coordinates = peak_local_max(
                    image_max, min_distance=current_min_distance)
                current_min_distance -= 1

            self.source_locations[m_type] = []

            for c in coordinates:
                self.source_locations[m_type].append([interpolator.lin_space_grid[m_type][0][c[0]],
                                                      interpolator.lin_space_grid[m_type][1][c[1]]])

    def publish_source_location(self):
        if self.pub_source_loc.get_num_connections() == 0:
            rospy.logwarn(
                "SourceLocalizationModule: Skipping source location publishing, because no subscribers have been found.")
            return

        # create markers
        marker_polygon_array = MarkerArray()

        for (m_type, locations) in list(self.source_locations.items()):
            m = Marker()
            m.header.stamp = rospy.Time()
            m.header.frame_id = self.params.global_frame
            m.ns = m_type + "_source_location"
            m.id = 0
            m.action = Marker.ADD
            m.type = Marker.SPHERE_LIST
            m.pose.orientation.w = 1.0
            m.scale.x = 0.5
            m.scale.y = 0.5
            m.scale.z = 0.5
            m.color.a = self.params.sl_marker_alpha

            generated_colors = []
            p_color = ColorRGBA()
            c = self.generate_new_color(generated_colors, pastel_factor=0.9)
            p_color.r = c[0]
            p_color.g = c[1]
            p_color.b = c[2]
            p_color.a = self.params.sl_marker_alpha

            for location in locations:
                p = Point()
                p.x = location[0]
                p.y = location[1]
                p.z = 1.0

                m.points.append(p)
                m.colors.append(p_color)

            marker_polygon_array.markers.append(m)

        self.pub_source_loc.publish(marker_polygon_array)

    def clear_markers(self):
        ma = MarkerArray()
        m = Marker()
        # m.header.stamp = rospy.Time()
        # m.header.frame_id = self.params.global_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.pub_source_loc.publish(ma)

    def get_random_color(self, pastel_factor=0.5):
        return [old_div((x+pastel_factor), (1.0+pastel_factor)) for x in [random.uniform(0, 1.0) for i in [1, 2, 3]]]

    def color_distance(self, c1, c2):
        return sum([abs(x[0]-x[1]) for x in zip(c1, c2)])

    def generate_new_color(self, existing_colors, pastel_factor=0.5):
        max_distance = None
        best_color = None
        for i in range(0, 100):
            color = self.get_random_color(pastel_factor=pastel_factor)
            if not existing_colors:
                return color
            best_distance = min([self.color_distance(color, c)
                                 for c in existing_colors])
            if not max_distance or best_distance > max_distance:
                max_distance = best_distance
                best_color = color
        return best_color
