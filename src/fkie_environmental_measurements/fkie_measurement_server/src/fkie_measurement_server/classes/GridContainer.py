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
from typing import Dict, List

from future import standard_library
from fkie_measurement_server.classes.IndexGrid import IndexGrid
standard_library.install_aliases()


class GridContainer(object):
    def __init__(self, grid_size):
        # type (float) -> None
        self.data = {}  # type: Dict[IndexGrid, List[float]]
        self.grid_size = grid_size  # type: float

    def add_to_grid(self, position, value):
        index = IndexGrid.position_to_index(position, self.grid_size)
        if index not in list(self.data.keys()):
            self.data[index] = []
        self.data[index].append(value)

    def get_values(self):
        # type: (...) -> List[List[float]]
        list_out = []  # type: List[List[float]]
        for index, values in list(self.data.items()):
            # # compute mean of all registered values
            # list_out.append([index.x, index.y, index.z, mean(values)])

            # compute max of all registered values
            position = IndexGrid.index_to_position(index, self.grid_size)
            list_out.append([position[0], position[1],
                             position[2], max(values)])
        return list_out

    def clear(self):
        self.data = {}
