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

from builtins import *
from builtins import object, range
from typing import List

import nav_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import rospy
import tf2_ros
import math
from fkie_measurement_server.ParameterManager import ParameterManager
from future import standard_library
from geometry_msgs.msg import Point
from threading import Lock
from std_msgs.msg import ColorRGBA

from fkie_measurement_server.classes.GridContainer import GridContainer

standard_library.install_aliases()


class TypedMeasurement(object):
    def __init__(self, source_type, unit, frame_id, grid_size, use_slam=False):
        # type: (str, str, str, float, bool) -> None
        self.use_slam = use_slam
        self.type = source_type
        self.unit = unit
        self.frame_id = frame_id
        self.lookup_mutex = Lock()

        # positions: dict{(x,y,z): [measurement_value, trajectory_index] ..}
        self.positions = dict()

        self.grid = GridContainer(grid_size)  # type: GridContainer

        self.__max_value = 0.0  # type: float
        self.__min_value = 999999999.0  # type: float

        # Get the params
        self.params = ParameterManager()
        if len(self.params.sensors) == 0:
            rospy.logerr("Could not find/update sensor configurations")
            return

        if self.use_slam:
            from cartographer_ros_msgs.srv import TrajectoryQuery, TrajectoryQueryResponse

            # Create publisher for trajectory
            self.traj_publisher = rospy.Publisher(
                'viz_traj_path', nav_msgs.msg.Path, queue_size=10)
            self.measurement_positions_publisher = rospy.Publisher(
                'viz_measurements_traj_markers', MarkerArray, queue_size=10)
            self.traj_marker_pub = rospy.Publisher(
                'viz_traj_marker', MarkerArray, queue_size=10)

            # Create the listener to query tfs
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

            # Create a timer that updates the trajectory index: sensor_position lookup table
            # {int: (x,y,z), ..}
            self.trajectory_lookup = dict()
            self.lookup_timer = rospy.Timer(rospy.Duration(
                self.params.slam_trajectory_lookup_update_interval), self.update_traj_lookup_cb)

            # Create a timer that updates position of the measurements periodically
            self.position_update_timer = rospy.Timer(rospy.Duration(
                self.slam_measurement_position_update_interval), self.update_positions_cb)

    def __repr__(self):
        # type: (...) -> str
        return self.__str__()

    def __str__(self):
        # type: (...) -> str
        return "Type: {0}, positions: {1}, min: {3}, max: {4}".format(
            self.type,
            len(self.positions),
            self.get_min_value(),
            self.get_max_value()
        )

    def get_min_value(self):
        # type: (...) -> float
        return self.__min_value

    def get_max_value(self):
        # type: (...) -> float
        return self.__max_value

    def add_measurement(self, position, stamp, value):
        if not self.use_slam:
            self.add_measurement_global(position, stamp, value)
        else:
            self.add_measurement_slam(position, stamp, value)

        if isinstance(value, float) and value > self.__max_value:
            self.__max_value = value

        if isinstance(value, float) and value < self.__min_value:
            self.__min_value = value

    # Methods without SLAM
    def add_measurement_global(self, position, stamp, value):
        self.positions[position] = [stamp, value, 0]
        self.grid.add_to_grid(position, value)

    # Methods used when SLAM is active
    def add_measurement_slam(self, position, stamp, value):
        # type: (List[float], rospy.Time(), float) -> None
        if not np.isnan(position).any() and not np.isnan(value):
            with self.lookup_mutex:
                rospy.logdebug(
                    "[add_measurement] Length of positions: %s" % len(self.positions))

                # If the lookup table is not empty, then find the index
                if len(self.trajectory_lookup) > 0:
                    rospy.logdebug("Lookup table has %s values, will look for the trajectory id of position (%s, %s, %s)" % (
                        len(self.trajectory_lookup), position[0], position[1], position[2]))

                    # Search for the nearest position and assign its trajectory index
                    nearest = self.trajectory_lookup[0][0]
                    new_id = -1

                    for key, key_val in list(self.trajectory_lookup.items()):
                        dis_to_pos = self.distance(
                            self.trajectory_lookup[key][0], position)

                        if dis_to_pos >= self.params.slam_matching_dist_threshold:
                            continue

                        if dis_to_pos <= self.distance(nearest, position):
                            nearest = self.trajectory_lookup[key][0]
                            new_id = key

                    self.positions[position] = [stamp, value, new_id]
                else:
                    rospy.logerr("[add_measurement] lookup table empty")
        else:
            rospy.logdebug("[add_measurement] invalid position or value")

    def update_traj_lookup_cb(self, event):
        # Call the service that queries the current trajectory, assume trajectory id to zero for now
        rospy.wait_for_service(self.params.slam_service_trajectory_query)
        response = TrajectoryQueryResponse()

        # For every trajectory index, transform the base_link in map to sensor_link in map
        try:
            client = rospy.ServiceProxy(
                self.params.slam_service_trajectory_query, TrajectoryQuery)
            response = client(self.params.slam_current_trajectory_id)

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: {0}".format(e))
            return

        if len(response.trajectory) > 0:
            # Query the position of the sensor in robot frame
            try:
                trans = self.tf_buffer.lookup_transform(
                    self.params.slam_robot_frame, self.frame_id, rospy.Time())
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logerr("[Timer: update_traj_lookup_cb] Could not find TF2 lookup between frames [{0}] and [{1}]".format(
                    self.params.slam_robot_frame, self.frame_id))
                return

            # Publish trajectory as marker array
            markers_to_publish = MarkerArray()

            for i in range(len(response.trajectory)):
                # Add the transformed pose to the dictionary
                with self.lookup_mutex:
                    new_position = (
                        response.trajectory[i].pose.position.x, response.trajectory[i].pose.position.y, response.trajectory[i].pose.position.z)

                    # in index exist, do not overwrite the TF between robot and sensor frame
                    if i in list(self.trajectory_lookup.keys()):
                        self.trajectory_lookup[i] = [
                            new_position, self.trajectory_lookup[i][1]]
                    else:
                        self.trajectory_lookup[i] = [new_position, trans]

                    # create point marker
                    m = Marker()
                    m.header.stamp = rospy.Time()
                    m.header.frame_id = self.params.global_frame
                    m.ns = "trajectory"
                    m.id = i
                    m.action = Marker.ADD
                    m.type = Marker.SPHERE_LIST
                    m.pose.orientation.w = 1.0
                    m.scale.x = 0.2
                    m.scale.y = 0.2
                    m.scale.z = 0.2
                    m.color.a = 0.8
                    m.color.r = 0.0
                    m.color.g = 0.0
                    m.color.b = 1.0

                    pp = Point()
                    pp.x = response.trajectory[i].pose.position.x
                    pp.y = response.trajectory[i].pose.position.y
                    pp.z = response.trajectory[i].pose.position.z
                    m.points.append(pp)

            markers_to_publish.markers.append(m)

            self.traj_marker_pub.publish(markers_to_publish)

            rospy.logdebug("Length of the lookup table dict: %s" %
                           len(self.trajectory_lookup))

        # Publish the trajectory for visualization
        msg = nav_msgs.msg.Path()
        msg.header.frame_id = self.params.global_frame
        msg.header.stamp = rospy.Time.now()
        msg.poses = response.trajectory
        self.traj_publisher.publish(msg)

    def distance(self, p1, p2):
        return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)

    def update_positions_cb(self, event):
        '''
        Using the trajectory index and position lookup table update the positions
        '''
        with self.lookup_mutex:

            if len(self.positions) == 0:
                return

            rospy.logdebug(
                "[update_positions_cb] Length of positions: %s" % len(self.positions))
            if len(self.trajectory_lookup) > 0:

                temp_copy = self.positions.copy()

                # Iterate over the copy and remove/add new keys if necessary.
                for position, val_index in list(temp_copy.items()):
                    measurement_stamp = temp_copy[position][0]
                    measurement_val = temp_copy[position][1]
                    traj_index = temp_copy[position][2]
                    rospy.logdebug(
                        "Current traj_index:%s and current position: (%s, %s, %s)" % (traj_index, position[0], position[1], position[2]))
                    # Check if the position's trajectory index exists in the lookup table
                    if traj_index in list(self.trajectory_lookup.keys()):
                        rospy.logdebug(
                            "Found traj_index %s in lookup table" % (traj_index))
                        new_position_base = self.trajectory_lookup[traj_index][0]

                        # apply robot to sensor transformation
                        trans = self.trajectory_lookup[traj_index][1]
                        new_position = (new_position_base[0] + trans.transform.translation.x,
                                        new_position_base[1] +
                                        trans.transform.translation.y,
                                        new_position_base[2] + trans.transform.translation.z)

                        del self.positions[position]
                        # Rewrite the key with the same values
                        self.positions[new_position] = [
                            measurement_stamp, measurement_val, traj_index]

                    # If id is invalid, then choose the nearest position's id.
                    elif traj_index == -1:
                        # Search for the nearest position and assign its trajectory index
                        nearest = self.trajectory_lookup[0][0]
                        new_id = -1
                        for traj_idx, pos in list(self.trajectory_lookup.items()):
                            dis_to_pos = self.distance(
                                self.trajectory_lookup[traj_idx][0], position)

                            if dis_to_pos >= self.params.slam_matching_dist_threshold:
                                continue

                            if dis_to_pos <= self.distance(nearest, position):
                                nearest = self.trajectory_lookup[traj_idx][0]
                                new_id = traj_idx

                        self.positions[position] = [
                            measurement_stamp, measurement_val, new_id]

            # update grid container
            self.grid.clear()
            for k, v in list(self.positions.items()):
                self.grid.add_to_grid(k, v[1])

            # create markers
            markers_to_publish = MarkerArray()

            # create point marker
            m = Marker()
            m.header.stamp = rospy.Time()
            m.header.frame_id = self.params.global_frame
            m.ns = "points"
            m.id = 0
            m.action = Marker.ADD
            m.type = Marker.SPHERE_LIST
            m.pose.orientation.w = 1.0
            m.scale.x = 0.2
            m.scale.y = 0.2
            m.scale.z = 0.2
            m.color.a = 0.8

            for k, v in list(self.positions.items()):
                pp = Point()
                pp.x = k[0]
                pp.y = k[1]
                pp.z = k[2]

                # Measurement points without valid indices appear as pink
                if self.positions[k][2] == -1:
                    c = ColorRGBA()
                    c.r = 1.0
                    c.g = 0.0
                    c.b = 0.7
                    c.a = 1.0
                else:
                    c = ColorRGBA()
                    c.r = 0.0
                    c.g = 1.0
                    c.b = 0.0
                    c.a = 1.0
                m.points.append(pp)
                m.colors.append(c)
            markers_to_publish.markers.append(m)
            self.measurement_positions_publisher.publish(markers_to_publish)
