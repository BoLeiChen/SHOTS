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

from builtins import *
from typing import Dict, Any

from future import standard_library

standard_library.install_aliases()


class Singleton(type):
    _instances = {}  # type: Dict[Any, Any]

    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(
                Singleton, cls).__call__(*args, **kwargs)
            #print(f"Singleton: New instance created: {cls}")
        # If you want to run __init__ every time the class is called
        # else:
        #     cls._instances[cls].__init__(*args, **kwargs)
        return cls._instances[cls]
