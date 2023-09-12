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
from builtins import object, range, str, zip

import actionlib
import fkie_measurement_msgs
import rospy
from fkie_measurement_server import Singleton
from fkie_measurement_server.MeasurementPublisher import MeasurementPublisher
from fkie_measurement_server.MeasurementServer import MeasurementServer
from fkie_measurement_server.ParameterManager import ParameterManager

from fkie_measurement_server.modules.BoundaryPolygonModule import BoundaryPolygonModule
from fkie_measurement_server.modules.InterpolationModule import InterpolationModule
from fkie_measurement_server.modules.SourceLocalizationModule import SourceLocalizationModule

from fkie_measurement_msgs.msg import BoundaryPolygon, MeasurementGaussianStatistics, MeasurementEstimationValue
from future import standard_library
from future.utils import with_metaclass
from geometry_msgs.msg import Point32
from std_msgs.msg import String

standard_library.install_aliases()


class MeasurementInterfaces(with_metaclass(Singleton, object)):
    def __init__(self):
        self.params = ParameterManager()

        self.m_measurement_server = MeasurementServer()
        self.m_publisher = MeasurementPublisher()
        self.m_interpolation = InterpolationModule()
        self.m_boundary_polygon = BoundaryPolygonModule()
        self.m_source_location = SourceLocalizationModule()

        self.request_save_measurements = False
        self.request_load_measurements = False
        self.request_clear_measurements = False
        self.request_compute_interpolation = False
        self.request_compute_boundary_polygons = False
        self.request_localize_sources = False

        self.sub_cmd = rospy.Subscriber(
            'commands', String, self.callback_command)

        # action server interfaces

        # RequestBoundaryPolygonsAction
        self.action_feedback = fkie_measurement_msgs.msg.RequestBoundaryPolygonsActionFeedback()
        self.action_result = fkie_measurement_msgs.msg.RequestBoundaryPolygonsResult()
        self.as_request_boundaries = actionlib.SimpleActionServer(
            "RequestBoundaryPolygonsAction",
            fkie_measurement_msgs.msg.RequestBoundaryPolygonsAction,
            execute_cb=self.as_request_boundaries_callback,
            auto_start=False)
        self.as_request_boundaries.start()

        # RequestGaussianInterpolation
        self.as_interpolation_feedback = fkie_measurement_msgs.msg.RequestGaussianInterpolationActionFeedback()
        self.as_interpolation_result = fkie_measurement_msgs.msg.RequestGaussianInterpolationResult()
        self.as_interpolation = actionlib.SimpleActionServer(
            "RequestGaussianInterpolationAction",
            fkie_measurement_msgs.msg.RequestGaussianInterpolationAction,
            execute_cb=self.as_interpolation_callback,
            auto_start=False)
        self.as_interpolation.start()

    def spin(self):
        # type: (...) -> None
        if self.request_save_measurements:
            self.m_measurement_server.save_measurements()
            self.request_save_measurements = False

        if self.request_load_measurements:
            self.m_measurement_server.clear_server()
            self.m_measurement_server.load_measurements()

            self.m_publisher.publish_measurements()
            self.request_load_measurements = False

        if self.request_clear_measurements:
            self.m_measurement_server.clear_server()
            self.m_publisher.publish_measurements()
            self.request_clear_measurements = False

        if self.request_compute_interpolation:
            self.m_interpolation.compute_interpolation()
            self.request_compute_interpolation = False

        if self.request_compute_boundary_polygons:
            self.m_interpolation.compute_interpolation(publish_marker=False)
            self.m_source_location.compute_source_location(
                self.m_interpolation)
            self.m_boundary_polygon.compute_boundary_polygons()
            self.m_boundary_polygon.publish_boundary_polygons()
            self.request_compute_boundary_polygons = False

        if self.request_localize_sources:
            self.m_interpolation.compute_interpolation(publish_marker=False)
            self.m_source_location.compute_source_location(
                self.m_interpolation)
            self.m_source_location.publish_source_location()
            self.request_localize_sources = False

        self.m_publisher.publish_measurements()

    def callback_command(self, msg):
        # type: (String) -> None
        rospy.loginfo("New command received: [{0}]".format(msg.data))

        if msg.data == "save_measurements":
            self.request_save_measurements = True

        if msg.data == "load_measurements":
            self.request_load_measurements = True

        if msg.data == "clear_measurements":
            self.request_clear_measurements = True

        if msg.data == "compute_interpolation":
            self.request_compute_interpolation = True

        if msg.data == "compute_boundary_polygons":
            self.request_compute_boundary_polygons = True

        if msg.data == "localize_sources":
            self.request_localize_sources = True

    def as_request_boundaries_callback(self, goal):
        # type: (fkie_measurement_msgs.msg.RequestBoundaryPolygonsGoal) -> None

        rospy.loginfo("New request to RequestBoundaryPolygonsAction")
        self.action_result.boundaries = []

        self.m_interpolation.compute_interpolation(publish_marker=False)
        self.m_source_location.compute_source_location(self.m_interpolation)
        self.m_boundary_polygon.compute_boundary_polygons()
        self.m_boundary_polygon.publish_boundary_polygons()

        if goal.source_type in list(self.m_boundary_polygon.polygons.keys()):
            contours = self.m_boundary_polygon.polygons[goal.source_type]
            for (i_contour, contour) in enumerate(contours):
                boundary = BoundaryPolygon()
                boundary.source_type = goal.source_type
                boundary.id = i_contour
                boundary.min_height = 0.0
                boundary.max_height = self.m_boundary_polygon.max_heights[goal.source_type]
                boundary.polygon.header.stamp = rospy.Time()
                boundary.polygon.header.frame_id = self.params.global_frame

                for i in range(0, len(contour)):
                    p = Point32()
                    p.x = contour[i][0]
                    p.y = contour[i][1]
                    boundary.polygon.polygon.points.append(p)

                self.action_result.boundaries.append(boundary)

        if goal.source_type in list(self.m_interpolation.interpolation_mesh_grid.keys()):
            self.action_result.estimations.header.frame_id = self.params.global_frame
            self.action_result.estimations.header.stamp = rospy.Time.now()
            self.action_result.estimations.grid_size = self.params.it_grid_size

            for (i_d, d) in enumerate(self.m_interpolation.interpolation_mesh_grid[goal.source_type]):
                [x, y, z, mean] = d

                mev = MeasurementEstimationValue()
                mev.x = x
                mev.y = y
                mev.z = z
                mev.mean = mean

                # add sigma if it was computed by the interpolator
                if goal.source_type in list(self.m_interpolation.interpolation_mesh_grid_variance.keys()):
                    [_, _, _, variance] = self.m_interpolation.interpolation_mesh_grid_variance[goal.source_type][i_d]
                    mev.variance = variance

                self.action_result.estimations.values.append(mev)

        self.as_request_boundaries.set_succeeded(self.action_result)

    def as_interpolation_callback(self, goal):
        # type: (fkie_measurement_msgs.msg.RequestGaussianInterpolationGoal) -> None

        rospy.loginfo("New request to RequestGaussianInterpolationAction")
        self.as_interpolation_result = fkie_measurement_msgs.msg.RequestGaussianInterpolationResult()

        # transform points for interpolation
        points = []
        for ps in goal.interpolation_points:
            try:
                ps_transformed = self.m_measurement_server.tf_buffer.transform(
                    ps, self.params.global_frame, rospy.Duration(3))
                points.append(
                    [ps_transformed.point.x, ps_transformed.point.y, ps_transformed.point.z])
            except Exception as error:
                rospy.logwarn(
                    "[as_interpolation_callback] Could not transform point between frames [{0}] and [{1}]. Ignoring point: {2}".format(
                        self.params.global_frame, ps.header.frame_id, error
                    ))
                self.as_interpolation.set_succeeded(
                    self.as_interpolation_result)
                return

        for source_type in goal.measurement_type_list:
            stat = MeasurementGaussianStatistics()
            stat.mean, stat.variance = self.m_interpolation.compute_gaussian_interpolation(
                points, source_type)
            self.as_interpolation_result.measurement_type_list.append(
                source_type)
            self.as_interpolation_result.statistics.append(stat)

            # publish markers with mean and variance (for debugging)
            self.m_interpolation.publish_gaussian_interpolation(
                source_type, points, stat.mean, stat.variance)

        self.as_interpolation.set_succeeded(self.as_interpolation_result)
