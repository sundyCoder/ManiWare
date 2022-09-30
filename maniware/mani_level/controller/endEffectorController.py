# Copyright 2022 IMCL, Department of Computing
# Department of Computing, Hong Kong Polytechnic University
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import pybullet as p
from .baseController import BaseController

class EndEffectController(BaseController):
    def __init__(self, end_effector):
        pass

    def release():
        pass

    def activate():
        pass


class SuckerController(BaseController):
    def __init__(self, sucker):
        pass

    def activate():
        pass

    def release():
        pass

class GripperController(BaseController):
    def __init__(self, gripper):
        pass

    def activate():
        pass

    def release():
        pass