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
from future import standard_library
standard_library.install_aliases()


class IndexGrid(object):
    def __init__(self, x, y, z):
        self.x = int(x)  # type: int
        self.y = int(y)  # type: int
        self.z = int(z)  # type: int

    def __hash__(self):
        return hash((self.x, self.y, self.z))

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y and self.z == other.z and self.__class__ == other.__class__

    @staticmethod
    def position_to_index(position, grid_size):
        # type: (List[float], float) -> IndexGrid
        x = int(round(position[0] / grid_size))
        y = int(round(position[1] / grid_size))
        z = int(round(position[2] / grid_size))
        return IndexGrid(x, y, z)

    @staticmethod
    def index_to_position(index, grid_size):
        # type: (IndexGrid, float) -> List[float]
        x = float(index.x * grid_size)
        y = float(index.y * grid_size)
        z = float(index.z * grid_size)
        return [x, y, z]
