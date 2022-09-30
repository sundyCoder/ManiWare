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

import os
import pybullet as p
from .baseTask import BaseTask

COLORS = {'red': [0.4, 0, 0], 'green': [0, 0.4, 0], 'blue': [0, 0, 0.4], 'black': [0, 0, 0], 'pink': [0.4, 0, 0.4],
          'yellow': [0.4, 0.4, 0], 'cyan': [0, 0.4, 0.4]}


class ManiTask(BaseTask):
    def __init__(self, env, position, thing_type, color='black'):
        self.env = env
        self.rtype = thing_type
        self.position = position
        self.init_position = position
        self.finished = False
        self.id = self._createBody()
        self.color_name = color
        self.color = COLORS[color]
        self.set_color(self.color_name)
        
        self.speed = None

    def _createBody(self):
        if self.rtype == "cube":
            cube_path = os.path.join(os.path.dirname(__file__), "../assets/conveyor/blue_cube.urdf")
            return p.loadURDF(cube_path, self.position, useFixedBase = 0)
        elif self.rtype == 'cylinder':
            cylinder_path = os.path.join(os.path.dirname(__file__), "../assets/conveyor/yellow_cylinder.urdf")
            return p.loadURDF(cylinder_path, self.position, useFixedBase = 0)


    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        self.position = position
        return self.position

    def set_color(self, color):
        self.color_name = color
        self.color = COLORS[color]
        p.changeVisualShape(self.id, 0, textureUniqueId=-1, rgbaColor=(self.color[0], self.color[1], self.color[2], 0.5), physicsClientId=self.env.client)


    def reset(self, pose):  #0.3
        position = pose[0]
        orientation = pose[1]
        p.resetBasePositionAndOrientation(self.id, position, orientation, physicsClientId=self.env.client)
        self.finished = False
        
    def _move(self):
        if self.speed:
            p.resetBaseVelocity(self.id, linearVelocity=[0., self.speed, 0.], angularVelocity=[0., 0., 0.], physicsClientId=self.env.client)


