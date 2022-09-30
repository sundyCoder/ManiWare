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
import numpy as np
from maniware.mani_level.component.baseChassis import BaseChassis

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
from ..controller.chassisController import MecanumController, DifferentialController


class Mecanum(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(MecanumController())

        self.fix_flag = False
        self.constrain_ground = None
        self.pose = pose

        self.lateral_velocity = 0.
        self.linear_velocity = 0.
        self.reached = False

    def _createBody(self):
        path = os.path.join(os.path.dirname(__file__), "../../assets/dingo/dingo-o.urdf")
        return p.loadURDF(path, self.pose[0], self.pose[1], useFixedBase=0)

    def fixed(self, pose=None):
        if not self.fix_flag:
            self.fix_flag = True
            if pose is None:
                position = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)[0]
                self.constrain_ground = p.createConstraint(self.env.planeID, -1, self.id, -1, p.JOINT_FIXED, None, position, [0., 0., 0.], self.init_pose[1])
            else:
                self.constrain_ground = p.createConstraint(self.env.planeID, -1, self.id, -1, p.JOINT_FIXED, None, pose[0], [0., 0., 0.], pose[1])

    def moved(self):
        if self.fix_flag:
            self.fix_flag = False
        if self.constrain_ground:
            p.removeConstraint(self.constrain_ground, physicsClientId=self.env.client)
            self.constrain_ground = None



    def step(self):
        self.update()
        self.reached = self.move_to(self.target_position, self.target_orientation)

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.lateral_velocity]
        return self.state

    def move_to(self, position, orientation):
        vx, vy, w, flag = self.controller.control_position(np.array(self.position), self.orientation, np.array(position), orientation)
        p.resetBaseVelocity(self.id, linearVelocity=[vx, vy, 0.], angularVelocity=[0., 0., w])
        # ?
        self.linear_velocity = vx * np.cos(self.orientation) + vy * np.sin(self.orientation)
        self.lateral_velocity = vy * np.cos(self.orientation) - vx * np.sin(self.orientation)
        if flag:
            return True
        else:
            return False



class Differential(BaseChassis):
    def __init__(self, env, pose):
        super().__init__(env, pose)
        self._set_controller(DifferentialController)

        self.linear_velocity = 0.
        self.angular_velocity = 0.

    def _createBody(self):
        pass

    def step(self):
        self.update()
        self.reached = self.move_to(self.target_position, self.target_orientation)

    def get_state(self):
        self.state = [self.position, self.orientation, self.linear_velocity, self.angular_velocity]
        return self.state

    def move_to(self, position, orientation):
        return False
