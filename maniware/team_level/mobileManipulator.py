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
from .executor import Executor
from maniware.mani_level.component.UR5 import UR5_new
from maniware.mani_level.component.chassis import Mecanum


class MobileManipulator(Executor):
    def __init__(self, env, position, ori, height, ee_type = 'Suction', chassis_type = 'Mecanum'):
        self.env = env

        self.position = position
        self.ori = ori
        self.height = height
        self.init_pose = None

        self.ur_base_pose = None

        self.ee_type = ee_type
        self.chassis_type = chassis_type
        self.rtype = ee_type

        self.ee = None
        self.chassis = None
        self.ur5 = None

        self.base_pose = 1      # base of arm

        self.reward = 0
        self._reward = 0

        self.state = None

        self.ee_tip = 10  #link id of end_effector

        self.action = None    # idle, goToGrab, grab, goToplace, place, idle
        self.last_action = None      # the action state that robot has just finished
        self.thing = None

        self.pick_pose = None   # the pose before it grab the task
        self.place_pose = None
        self.waitNum = 0

        self.target_position = None
        self.move_flag = True

        self._createBody()

    def _createBody(self):
        if self.chassis_type:
            pose = ((self.position[0], self.position[1], self.position[2]), p.getQuaternionFromEuler((0., 0., self.ori)))
            self.init_pose = ((self.position[0], self.position[1], self.position[2]), p.getQuaternionFromEuler((0., 0., self.ori)))
            if self.chassis_type == 'Mecanum':
                self.chassis = Mecanum(self.env, pose)
            self.chassis.fixed()
            self.ur_base_pose = ((self.position[0], self.position[1], self.position[2]+0.1), p.getQuaternionFromEuler((0., 0., self.ori + 0.)))
            self.ur5 = UR5_new(self.env, self.ur_base_pose, 0, self.ee_type, self.height)
            self.chassis_ur5_constraint = p.createConstraint(self.chassis.id, -1, self.ur5.id, -1, p.JOINT_FIXED, None,
                                                             [0., 0., 0.1], [0., 0., 0.])
        else:
            self.ur5 = UR5_new(self.env, self.ur_base_pose, 1, self.ee_type, self.height)

    def set_chassis(self, chassis_type):
        self.chassis_type = chassis_type
        if self.chassis:
            p.removeConstraint(self.chassis_ur5_constraint)
            self.chassis.remove()
        self.chassis = Mecanum(self.env, self.position)
        self.chassis_ur5_constraint = p.createConstraint(self.chassis.id, -1, self.ur5.id, -1, p.JOINT_FIXED, None,
                                                         self.chassis.position, [0., 0., 0.])



    def reset(self):
        if self.chassis:
            self.chassis.moved()
            # self.chassis.set_init_pose(self.init_pose)
            self.chassis.fixed(self.chassis.init_pose)
            print(self.chassis.init_pose)


        self.ur5.reset()
        self.ur5.ee.release()

        self.reward = 0  # Cumulative returned rewards.
        self.action = 'idle'   # move, idle, grab
        self.working = False

    def move_collect(self):
        if self.action == 'idle' and self.thing:
            self.chassis.fixed()
            self.chassis.set_target(self.thing.get_position(), self.chassis.orientation)
            # self.ur5.step()
            self.action = 'move'
            self.last_action = 'idle'
            # self.thing = thing
        elif self.action == 'move':
            self.chassis.moved()
            self.chassis.step()
            self.position = self.chassis.position
            # self.ur5.step()
            if self.chassis.reached:
                self.action = 'grab'
                self.last_action = 'move'
                self.chassis.fixed()
                self.ur5.thing = self.thing
        elif self.action == 'grab':
            self.ur5.pick_and_place_FSM()
            if self.ur5.grab_finished and self.ur5.place_position is None:
                self.ur5.place_position = np.array([self.chassis.position[0] - 0.3 * np.cos(self.chassis.orientation),
                                                    self.chassis.position[1] - 0.3 * np.sin(self.chassis.orientation),
                                                    self.chassis.position[2] + 0.5])

            if self.thing not in self.env.available_thing_ids_set:
                self.action = 'waiting'
                self.last_action = 'grab'
        elif self.action == 'waiting':
            self.waitNum += 1
            self.ur5.step()
            if self.waitNum >= 300:
                self.waitNum = 0
                self.chassis.moved()
                self.action = 'idle'
                self.last_action = 'waiting'
                self.thing = None


    def get_state(self):
        return self.state

    def reward(self):
        info = {}
        if self._reward > 0:
            self.reward += self._reward
            self._reward = 0
            info['success'] = 1
            return self.reward, info, 1
        else:
            info['success'] = 0
            return self.reward, info, 0

