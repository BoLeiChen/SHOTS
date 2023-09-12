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
from builtins import object, range, str, zip
from typing import Any, Dict, List, Tuple

import numpy as np
import rospy
import scipy
from fkie_measurement_server import Singleton
from fkie_measurement_server.MeasurementServer import MeasurementServer
from fkie_measurement_server.ParameterManager import ParameterManager
from fkie_measurement_server.utils import *
from future import standard_library
from future.utils import with_metaclass
from geometry_msgs.msg import Point
from numpy import ndarray
from visualization_msgs.msg import Marker, MarkerArray

from sklearn.gaussian_process.kernels import RBF
from sklearn.gaussian_process import GaussianProcessRegressor

standard_library.install_aliases()


class InterpolationModule(with_metaclass(Singleton, object)):
    def __init__(self):
        self.params = ParameterManager()
        self.ms = MeasurementServer()

        self.grid_mesh = {}  # type: Dict[str, np.array]
        # type: Dict[str, Tuple[np.array, np.array, np.array]]
        self.lin_space_grid = {}

        # limits for output grid generation
        self.boundary_min = [-1e10, -1e10, -1e10]
        self.boundary_max = [1e10, 1e10, 1e10]

        # kriging objects
        self.k3d = {}  # type: Dict[str, ndarray]
        self.ss3d = {}  # type: Dict[str, ndarray]
        self.ok3d = None  # type: OrdinaryKriging3D
        self.uk3d = None  # type: UniversalKriging3D

        self.interpolation_mesh_grid = {}  # type: Dict[str, List[List[float]]]
        # type: Dict[str, List[List[float]]]
        self.interpolation_mesh_grid_variance = {}
        # Dict: type, [x, y, max_value]
        self.max_mesh_grid = {}  # type: Dict[str, List[List[float]]]

        self.gaussian_kernel = RBF(
            length_scale=10.0, length_scale_bounds="fixed")  # RBF kernel
        self.gaussian_gp = GaussianProcessRegressor(
            kernel=self.gaussian_kernel, alpha=0.01,   normalize_y=True)
        
        # ROS interface (publisher/subscribers)
        self.pub_interpolation = rospy.Publisher(
            "interpolation_marker", MarkerArray, queue_size=5, latch=True)
        self.pub_gaussian = rospy.Publisher(
            "gaussian", MarkerArray, queue_size=5, latch=True)

        self.clear_markers()

    def compute_interpolation(self, publish_marker=True):
        # type: (bool) -> None

        if self.create_mesh_grid():
            if self.params.it_method == "LinearNDInterpolator":
                self.compute_linear_interpolation()

            elif self.params.it_method == "NearestNDInterpolator":
                self.compute_nearest_interpolation()

            elif self.params.it_method == "RBF":
                self.compute_rbf_interpolation()

            elif self.params.it_method == "OrdinaryKriging3D":
                self.compute_kriging_interpolation()

            elif self.params.it_method == "GaussianRegression":
                self.compute_gaussian_regression()

            else:
                rospy.logerr(
                    "Unsupported interpolation method [{0}]".format(self.params.it_method))

            rospy.loginfo("compute_interpolation ready!")
        else:
            rospy.logerr("Could not create mesh grid for interpolation")

        if publish_marker:
            self.publish_interpolation()

    def create_mesh_grid(self):
        # type: (...) -> bool
        if len(self.ms.data) == 0:
            rospy.logwarn("InterpolationModule: Measurements are empty")
            return False

        self.boundary_min = [1e10, 1e10, 1e10]
        self.boundary_max = [-1e10, -1e10, -1e10]

        # prepare data
        for m_type, m_data in list(self.ms.data.items()):
           # get bounding limits: get minimun per axis to define interpolation bounding box
            p = list(m_data.positions.keys())
            p_list = list(zip(*p))

            self.boundary_min = [
                min(p_list[0]), min(p_list[1]), min(p_list[2])]
            self.boundary_max = [
                max(p_list[0]), max(p_list[1]), max(p_list[2])]

            padding = (self.params.it_grid_size + self.params.it_padding)

            X = np.arange(
                start=self.boundary_min[0] - padding,
                stop=self.boundary_max[0] + padding,
                step=self.params.it_grid_size)

            Y = np.arange(
                start=self.boundary_min[1] - padding,
                stop=self.boundary_max[1] + padding,
                step=self.params.it_grid_size)

            Z = np.arange(
                start=self.boundary_min[2] - padding,
                stop=self.boundary_max[2] + padding,
                step=self.params.it_grid_size)

            self.lin_space_grid[m_type] = (X, Y, Z)
            self.grid_mesh[m_type] = np.meshgrid(X, Y, Z)

        return len(self.lin_space_grid) > 0 and len(self.grid_mesh) > 0

    def publish_interpolation(self):
        # type: (...) -> None
        if self.pub_interpolation.get_num_connections() == 0:
            rospy.logwarn(
                "InterpolationModule: Skipping interpolation publishing, because no subscribers have been found.")
            return

        self.clear_markers()

        if len(self.interpolation_mesh_grid) == 0 or len(self.max_mesh_grid) == 0:
            rospy.logwarn("InterpolationModule: Invalid mesh grid.")
            return

        self.ms.data_lock.acquire()
        try:
            # create markers
            markers_to_publish = MarkerArray()
            for m_type, m_data in list(self.ms.data.items()):
                # create voxel marker
                m = Marker()
                m.header.stamp = rospy.Time()
                m.header.frame_id = self.params.global_frame
                m.ns = m_type + "_voxels"
                m.id = 0
                m.action = Marker.ADD
                m.type = Marker.CUBE_LIST
                m.pose.orientation.w = 1.0
                m.scale.x = self.params.it_grid_size
                m.scale.y = self.params.it_grid_size
                m.scale.z = self.params.it_grid_size
                m.color.a = self.params.it_marker_alpha

                if m_type not in list(self.interpolation_mesh_grid.keys()):
                    continue

                for d in self.interpolation_mesh_grid[m_type]:
                    [x, y, z, value] = d
                    pp = Point()
                    pp.x = x + (self.params.it_grid_size / 2.0)
                    pp.y = y + (self.params.it_grid_size / 2.0)
                    pp.z = z + (self.params.it_grid_size / 2.0)

                    c = compute_color(
                        value, m_data.get_min_value(), m_data.get_max_value())
                    # c.a = self.params.it_marker_alpha

                    m.points.append(pp)
                    m.colors.append(c)

                markers_to_publish.markers.append(m)

                # create variance marker
                if m_type in list(self.interpolation_mesh_grid_variance.keys()):
                    m_var = Marker()
                    m_var.header.stamp = rospy.Time()
                    m_var.header.frame_id = self.params.global_frame
                    m_var.ns = m_type + "_sigma_voxels"
                    m_var.id = 1
                    m_var.action = Marker.ADD
                    m_var.type = Marker.CUBE_LIST
                    m_var.pose.orientation.w = 1.0
                    m_var.scale.x = self.params.it_grid_size
                    m_var.scale.y = self.params.it_grid_size
                    m_var.scale.z = self.params.it_grid_size
                    m_var.color.a = self.params.it_marker_alpha

                    l_var = np.concatenate(
                        np.array(self.interpolation_mesh_grid_variance[m_type])[:, 3], axis=0)
                    min_val = np.min(l_var)
                    max_val = np.max(l_var)

                    for d in self.interpolation_mesh_grid_variance[m_type]:
                        [x, y, z, value] = d
                        pp = Point()
                        pp.x = x + (self.params.it_grid_size / 2.0)
                        pp.y = y + (self.params.it_grid_size / 2.0)
                        pp.z = z + (self.params.it_grid_size / 2.0)

                        c = compute_color(value, min_val, max_val)
                        # c.a = self.params.it_marker_alpha

                        m_var.points.append(pp)
                        m_var.colors.append(c)

                    if len(m_var.points) > 0:
                        markers_to_publish.markers.append(m_var)

                # create max marker
                m_max = Marker()
                m_max.header.stamp = rospy.Time()
                m_max.header.frame_id = self.params.global_frame
                m_max.ns = m_type + "_max"
                m_max.id = 2
                m_max.action = Marker.ADD
                m_max.type = Marker.CUBE_LIST
                m_max.pose.orientation.w = 1.0
                m_max.scale.x = self.params.it_grid_size
                m_max.scale.y = self.params.it_grid_size
                m_max.scale.z = 0.1
                m_max.color.a = self.params.it_marker_alpha

                for d in self.max_mesh_grid[m_type]:
                    [x, y, value] = d
                    pp = Point()
                    pp.x = x + (self.params.it_grid_size / 2.0)
                    pp.y = y + (self.params.it_grid_size / 2.0)
                    pp.z = 0.0

                    c = compute_color(
                        value, m_data.get_min_value(), m_data.get_max_value())
                    # c.a = self.params.it_marker_alpha

                    if c.a < self.params.it_min_alpha:
                        continue  # skip almost zero markers

                    m_max.points.append(pp)
                    m_max.colors.append(c)

                if len(m_max.points) > 0:
                    markers_to_publish.markers.append(m_max)
        finally:
            self.ms.data_lock.release()

        self.pub_interpolation.publish(markers_to_publish)

    def compute_linear_interpolation(self):
        # type: (...) -> None

        rospy.loginfo("Computing linear interpolation")

        self.interpolation_mesh_grid = {}
        self.max_mesh_grid = {}

        for m_type, m_data in list(self.ms.data.items()):
            grid_data = np.array(m_data.grid.get_values())
            positions = grid_data[:, (0, 1, 2)]
            values = grid_data[:, 3]
            interpolator = scipy.interpolate.LinearNDInterpolator(
                points=positions,
                values=values,
                fill_value=m_data.get_min_value(),
                rescale=self.params.it_LinearNDInterpolator_rescale)

            grid = []  # type: List[List[float]]
            max_r = []  # type: List[List[float]]
            for x in self.lin_space_grid[m_type][0]:
                for y in self.lin_space_grid[m_type][1]:
                    max_v = m_data.get_min_value()
                    for z in self.lin_space_grid[m_type][2]:
                        v = float(interpolator(x, y, z))

                        # bound interpolated values
                        v = v if v > m_data.get_min_value() else m_data.get_min_value()
                        v = v if v < m_data.get_max_value() else m_data.get_max_value()

                        # ignore min interpolation values
                        if v < self.params.it_min_mean_value:
                            continue

                        grid.append([x, y, z, v])
                        if v > max_v:  # get maximum "2.5D" value
                            max_v = v
                    max_r.append([x, y, max_v])

            self.interpolation_mesh_grid[m_type] = grid
            self.max_mesh_grid[m_type] = max_r

    def compute_gaussian_regression(self):
        # type: (...) -> None
        self.interpolation_mesh_grid = {}
        self.interpolation_mesh_grid_variance = {}
        self.max_mesh_grid = {}

        self.ms.data_lock.acquire()
        try:
            for m_type, m_data in list(self.ms.data.items()):
                grid_data = np.array(m_data.grid.get_values())
                positions = grid_data[:, (0, 1, 2)]
                values = grid_data[:, 3]

                # Fit to data using Maximum Likelihood Estimation of the parameters
                self.gaussian_gp.fit(positions, values)

                # "log_marginal_likelihood: {0}".format(self.gaussian_gp.log_marginal_likelihood()))

                grid = []  # type: List[List[float]]
                grid_variance = []  # type: List[List[float]]
                max_r = []  # type: List[List[float]]
                for x in self.lin_space_grid[m_type][0]:
                    for y in self.lin_space_grid[m_type][1]:
                        max_v = m_data.get_min_value()
                        for z in self.lin_space_grid[m_type][2]:
                            # TODO: Improve this by using array instead of single samples!
                            v, sigma = self.gaussian_gp.predict(
                                [[x, y, z]], return_cov=True)  # return_std=True)

                            # bound interpolated values
                            v = v if v > m_data.get_min_value() else m_data.get_min_value()
                            v = v if v < m_data.get_max_value() else m_data.get_max_value()

                            # ignore min interpolation values
                            if v < self.params.it_min_mean_value:
                                continue

                            grid.append([x, y, z, v])
                            grid_variance.append([x, y, z, sigma])
                            if v > max_v:  # get maximum "2.5D" value
                                max_v = v
                        max_r.append([x, y, max_v])

                self.interpolation_mesh_grid[m_type] = grid
                self.interpolation_mesh_grid_variance[m_type] = grid_variance
                self.max_mesh_grid[m_type] = max_r
        finally:
            self.ms.data_lock.release()

    def compute_nearest_interpolation(self):
        # type: (...) -> None

        rospy.loginfo("Computing nearest interpolation")

        self.interpolation_mesh_grid = {}
        self.max_mesh_grid = {}

        for m_type, m_data in list(self.ms.data.items()):
            grid_data = np.array(m_data.grid.get_values())
            positions = grid_data[:, (0, 1, 2)]
            values = grid_data[:, 3]

            interpolator = scipy.interpolate.NearestNDInterpolator(
                x=positions,
                y=values,
                rescale=self.params.it_NearestNDInterpolator_rescale)

            grid = []
            max_r = []
            for x in self.lin_space_grid[m_type][0]:
                for y in self.lin_space_grid[m_type][1]:
                    max_v = m_data.get_min_value()
                    for z in self.lin_space_grid[m_type][2]:
                        v = float(interpolator(x, y, z))

                        # bound interpolated values
                        v = v if v > m_data.get_min_value() else m_data.get_min_value()
                        v = v if v < m_data.get_max_value() else m_data.get_max_value()

                        # ignore min interpolation values
                        if v < self.params.it_min_mean_value:
                            continue

                        grid.append([x, y, z, v])
                        if v > max_v:
                            max_v = v
                    max_r.append([x, y, max_v])

            self.interpolation_mesh_grid[m_type] = grid
            self.max_mesh_grid[m_type] = max_r

    def compute_rbf_interpolation(self):
        # type: (...) -> None
        rospy.loginfo("Computing RBF interpolation")

        self.interpolation_mesh_grid = {}
        self.max_mesh_grid = {}

        for m_type, m_data in list(self.ms.data.items()):
            grid_data = np.array(m_data.grid.get_values())
            if len(grid_data) < 1:
                continue

            positions = grid_data[:, (0, 1, 2)]
            values = grid_data[:, 3]

            try:
                interpolator = scipy.interpolate.Rbf(
                    positions[:, 0], positions[:, 1], positions[:, 2], values,
                    function=self.params.it_RBF_function)
                grid = []
                max_r = []
                for x in self.lin_space_grid[m_type][0]:
                    for y in self.lin_space_grid[m_type][1]:
                        max_v = m_data.get_min_value()
                        for z in self.lin_space_grid[m_type][2]:
                            v = float(interpolator(x, y, z))

                            # bound interpolated values
                            v = v if v > m_data.get_min_value() else m_data.get_min_value()
                            v = v if v < m_data.get_max_value() else m_data.get_max_value()

                            # ignore min interpolation values
                            if v < self.params.it_min_mean_value:
                                continue

                            grid.append([x, y, z, v])
                            if v > max_v:
                                max_v = v
                        max_r.append([x, y, max_v])

                self.interpolation_mesh_grid[m_type] = grid
                self.max_mesh_grid[m_type] = max_r
            except BaseException as e:
                rospy.logerr("RBF error: {0}".format(str(e)))

    def compute_kriging_interpolation(self):
        # type: (...) -> None
        rospy.loginfo("Computing Kriging interpolation")

        igs = self.params.it_grid_size
        self.clear_kriging()

        self.interpolation_mesh_grid = {}
        self.max_mesh_grid = {}

        # prepare data
        for m_type, m_data in list(self.ms.data.items()):
            grid_data = np.array(m_data.grid.get_values())
            p = grid_data[:, (0, 1, 2)]
            v = grid_data[:, 3]

            # define kriging grid
            grid_x = self.lin_space_grid[m_type][0]
            grid_y = self.lin_space_grid[m_type][1]
            grid_z = self.lin_space_grid[m_type][2]

            if len(grid_x) == 0 or len(grid_y) == 0 or len(grid_z) == 0:
                continue

            # compute interpolation
            try:
                # style: Specifies how to treat input kriging points.
                # ‘grid’ treats points as arrays of x, y, and z coordinates that define a rectangular grid.
                # ‘points’ treats points as arrays that provide coordinates at which to solve the kriging system.
                # ‘masked’ treats points as arrays of x, y, and z coordinates that define a rectangular grid and uses mask to only evaluate specific points in the grid.
                kriging_style = 'grid'
                if self.params.it_OrdinaryKriging3D_kriging_type == "ordinary":
                    from pykrige.ok3d import OrdinaryKriging3D
                    self.ok3d = OrdinaryKriging3D(
                        x=p[:, 0], y=p[:, 1], z=p[:, 2], val=v,
                        variogram_model=self.params.it_OrdinaryKriging3D_variogram_model)
                    self.k3d[m_type], self.ss3d[m_type] = self.ok3d.execute(
                        kriging_style, grid_x, grid_y, grid_z,
                        mask=np.zeros((len(grid_x), len(grid_y), len(grid_z)), dtype=bool))

                elif self.params.it_OrdinaryKriging3D_kriging_type == "universal":
                    from pykrige.uk3d import UniversalKriging3D
                    self.uk3d = UniversalKriging3D(
                        p[:, 0], p[:, 1], p[:, 2], v,
                        variogram_model=self.params.it_OrdinaryKriging3D_variogram_model)
                    self.k3d[m_type], self.ss3d[m_type] = self.uk3d.execute(
                        kriging_style, grid_x, grid_y, grid_z,
                        mask=np.zeros((len(grid_x), len(grid_y), len(grid_z)), dtype=bool))

                # prepare output data
                max_value = m_data.get_max_value()

                grid = []
                max_r = []
                for (xi, x) in enumerate(grid_x):
                    for (yi, y) in enumerate(grid_y):
                        max_value_z = m_data.get_min_value()
                        for (zi, z) in enumerate(grid_z):
                            v = float(self.k3d[m_type].data[zi][yi][xi])

                            # bound interpolated values
                            v = v if v > m_data.get_min_value() else m_data.get_min_value()
                            v = v if v < m_data.get_max_value() else m_data.get_max_value()

                            # ignore min interpolation values
                            if v < self.params.it_min_mean_value:
                                continue

                            grid.append([x, y, z, v])

                            if v > max_value_z:
                                max_value_z = v
                        max_r.append([x, y, max_value_z])

                self.interpolation_mesh_grid[m_type] = grid
                self.max_mesh_grid[m_type] = max_r

            except BaseException as e:
                rospy.logerr("Kriging error: {0} ".format(str(e)))
                self.clear_kriging()

    def clear_kriging(self):
        # type: (...) -> None

        # kriging objects
        self.k3d = {}
        self.ss3d = {}
        self.ok3d = None
        self.uk3d = None

    def clear_markers(self):
        # type: (...) -> None

        ma = MarkerArray()
        m = Marker()
        m.header.stamp = rospy.Time()
        m.header.frame_id = self.params.global_frame
        m.action = Marker.DELETEALL
        ma.markers.append(m)
        self.pub_interpolation.publish(ma)

    def compute_gaussian_interpolation(self, points, m_type):
        # type: (Any, Any, str) -> Tuple[List[float], List[float], List[float], List[float], List[float]]
        mean = []
        variance = []

        if len(points) < 1:
            rospy.logwarn(
                "compute_gaussian_interpolation: too few points for computing interpolation")
            return mean, variance

        if m_type not in list(self.ms.data.keys()):
            rospy.logerr(
                "compute_gaussian_interpolation: Ignoring invalid type: " + m_type)
            return mean, variance

        self.ms.data_lock.acquire()
        try:
            # extract values for interpolation
            grid_data = np.array(self.ms.data[m_type].grid.get_values())

            if len(grid_data) < 1:
                rospy.logerr(
                    "compute_gaussian_interpolation: grid.get_values() returns empty list")
                return mean, variance

            positions = grid_data[:, (0, 1, 2)]
            values = grid_data[:, 3]

            # Fit to data using Maximum Likelihood Estimation of the parameters
            self.gaussian_gp.fit(positions, values)
            m_array, v_array = self.gaussian_gp.predict(
                points, return_std=True)

            for i in range(len(m_array)):
                m = m_array[i]
                v = v_array[i]

                # bound interpolated values
                m = m if m > self.ms.data[m_type].get_min_value(
                ) else self.ms.data[m_type].get_min_value()
                m = m if m < self.ms.data[m_type].get_max_value(
                ) else self.ms.data[m_type].get_max_value()

                # TODO: normalize results?
                # n = (value - min_value) / (max_value - min_value)
                m = (m - self.ms.data[m_type].get_min_value()) / (
                    self.ms.data[m_type].get_max_value() - self.ms.data[m_type].get_min_value())

                mean.append(m)
                variance.append(v)

        finally:
            self.ms.data_lock.release()

        return mean, variance

    def publish_gaussian_interpolation(self, m_type, points, mean, variance):

        if m_type not in list(self.ms.data.keys()):
            rospy.logerr(
                "publish_gaussian_interpolation: Ignoring invalid type: " + m_type)
            return

        if len(mean) < 1 or len(variance) < 1:
            rospy.logerr(
                "publish_gaussian_interpolation: empty mean/variance ")
            return

        # create markers
        markers_to_publish = MarkerArray()

        # create mean marker
        m = Marker()
        m.header.stamp = rospy.Time()
        m.header.frame_id = self.params.global_frame
        m.ns = m_type + "_mean"
        m.id = 0
        m.action = Marker.ADD
        m.type = Marker.CUBE_LIST
        m.pose.orientation.w = 1.0
        m.scale.x = self.params.it_grid_size
        m.scale.y = self.params.it_grid_size
        m.scale.z = self.params.it_grid_size
        m.color.a = self.params.it_marker_alpha

        # create variance marker
        m_max = Marker()
        m_max.header.stamp = rospy.Time()
        m_max.header.frame_id = self.params.global_frame
        m_max.ns = m_type + "_variance"
        m_max.id = 1
        m_max.action = Marker.ADD
        m_max.type = Marker.CUBE_LIST
        m_max.pose.orientation.w = 1.0
        m_max.scale.x = self.params.it_grid_size / 2.0
        m_max.scale.y = self.params.it_grid_size / 2.0
        m_max.scale.z = self.params.it_grid_size / 2.0
        m_max.color.a = self.params.it_marker_alpha

        variance_a = np.sqrt(np.array(variance))

        # min_mean = self.ms.data[m_type].get_min_value()
        # max_mean = self.ms.data[m_type].get_max_value()

        min_mean = 0.0
        max_mean = 1.0

        min_variance = np.min(variance_a)
        max_variance = np.max(variance_a)

        for i in range(len(points)):
            [x, y, z, mean_val, variance_val] = [points[i][0], points[i]
                                                 [1], points[i][2], mean[i], np.sqrt(variance[i])]
            pp = Point()
            pp.x = x + (self.params.it_grid_size / 2.0)
            pp.y = y + (self.params.it_grid_size / 2.0)
            pp.z = z + (self.params.it_grid_size / 2.0)

            c = compute_color(mean_val, min_mean, max_mean)

            m.points.append(pp)
            m.colors.append(c)

            m_max.points.append(pp)

            if variance_val >= min_variance:
                c_v = compute_color(variance_val, min_variance, max_variance)
                m_max.colors.append(c_v)
            else:
                c_v = get_color("white")
                m_max.colors.append(c_v)

        markers_to_publish.markers.append(m)
        markers_to_publish.markers.append(m_max)

        self.pub_gaussian.publish(markers_to_publish)
