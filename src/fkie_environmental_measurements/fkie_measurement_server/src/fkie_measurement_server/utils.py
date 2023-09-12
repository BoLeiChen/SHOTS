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
from typing import Any

from future import standard_library
from past.utils import old_div
from std_msgs.msg import ColorRGBA

import rospy

standard_library.install_aliases()

_RED_COLOR = ColorRGBA()
_RED_COLOR.r = 1.0
_RED_COLOR.g = 0.0
_RED_COLOR.b = 0.0
_RED_COLOR.a = 1.0

_WHITE_COLOR = ColorRGBA()
_WHITE_COLOR.r = 1.0
_WHITE_COLOR.g = 1.0
_WHITE_COLOR.b = 1.0
_WHITE_COLOR.a = 1.0

_GRAY_COLOR = ColorRGBA()
_GRAY_COLOR.r = 0.8
_GRAY_COLOR.g = 0.8
_GRAY_COLOR.b = 0.8
_GRAY_COLOR.a = 1.0


def interpolate_color(color_a, color_b, fraction):
    # type: (ColorRGBA, ColorRGBA, float) -> ColorRGBA
    c = ColorRGBA()
    c.r = color_a.r + (color_b.r - color_a.r) * fraction
    c.g = color_a.g + (color_b.g - color_a.g) * fraction
    c.b = color_a.b + (color_b.b - color_a.b) * fraction
    c.a = color_a.a + (color_b.a - color_a.a) * fraction
    return c


def get_color(name):
    if name == "red":
        return _RED_COLOR

    if name == "white":
        return _WHITE_COLOR

    if name == "gray":
        return _GRAY_COLOR


def compute_color(value,  min_value, max_value):
    # type: (float, float, float) -> ColorRGBA
    if max_value - min_value != 0:
        normalized = old_div((value - min_value), (max_value - min_value))
    else:
        normalized = 0.0

    c = ColorRGBA()
    if normalized > 1.0 or normalized < 0.0:
        rospy.logwarn(
            "[compute_color] Error at normalizing value [{0}], between min [{1}] and max [{2}]. Normalized: [{3}]".format(
                value, min_value, max_value, normalized)
        )
        return c

    # interpolate between red and white
    c = interpolate_color(_WHITE_COLOR, _RED_COLOR, normalized)

    # set alpha
    c.a = normalized
    return c
