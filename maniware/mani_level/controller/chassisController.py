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

def get_euler_from_quaternion(quaternion):
    return list(p.getEulerFromQuaternion(quaternion))

def restrain(theta):
    if theta > np.pi:
        theta -= 2*np.pi
    elif theta < -np.pi:
        theta += 2*np.pi
    return theta

class ChassisController(BaseController):
    def __init__(self, chassis):
        pass

    def control_velocity():
        pass

    def control_position():
        pass

class MecanumController(BaseController):
    def __init__(self):
        self.velocity = None

    def control(self, linear_velocity, lateral_velocity):
        return linear_velocity, lateral_velocity

    def control_position(self, cur_position, cur_orientation, position, orientation):
        delta_position = position - cur_position
        delta_orientation = restrain(orientation - cur_orientation)
        distance = np.sqrt(delta_position[0]**2 + delta_position[1]**2)
        v = max(min(0.5 * distance / 3., 0.5), 0.15)
        vx = v * delta_position[0] / distance
        vy = v * delta_position[1] / distance
        w = delta_orientation
        if distance < 0.7:
            if abs(w) < 0.2:
                return 0., 0., 0., True
            else:
                return 0., 0., 0., True
        return vx, vy, 0., False

class DifferentialController(BaseController):
    def __init__(self):
        pass