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

import pybullet as p
from .baseComponent import BaseComponent

def get_euler_from_quaternion(quaternion):
    return list(p.getEulerFromQuaternion(quaternion))

class BaseChassis(BaseComponent):
    def __init__(self, env, pose):
        self.env = env
        self.pose = pose
        self.init_pose = self.pose

        self._last_step_simulation_count = -1
        self.position = self.pose[0]
        self.orientation = get_euler_from_quaternion(self.pose[1])[2]

        self.state = None

        self.target_position = None
        self.target_orientation = None
        self.action = None
        self.awaiting_action = False

        self.waypoint_pos = None
        self.waypoint_ori = None

        self.controller = None

        self.collision = False   #set([self.id])
        self.collision_obstacle = False
        self.collision_vehicle = False

        self.id = self._createBody()

    def update(self):
        self.pose = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        self.position = self.pose[0]
        self.orientation = get_euler_from_quaternion(self.pose[1])[2]

    def _createBody(self):
        pass

    def _set_controller(self, controller):
        self.controller = controller

    def set_target(self, position, orientation):
        self.target_position = position
        self.target_orientation = orientation

    def set_init_position(self, pose):
        self.init_pose = pose

    def get_position(self):
        position, _ = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        self.position = position
        return self.position

    def step(self):
        pass

    def get_state(self):
        pass

    def set_init_pose(self, pose):
        self.init_pose = pose

    def reset(self):
        self.action = None
        self.target_position = None
        self.waypoint_pos = None
        self.waypoint_ori = None
        self.reset_pose(self.init_pose)

    def reset_pose(self, pose):
        p.resetBasePositionAndOrientation(self.id, pose[0], pose[1])
        self._last_step_simulation_count = -1

    def check_for_collisions(self):
        for contact_point in self.env.p.getContactPoints(self.id):
            body_b_id = contact_point[2]
            if body_b_id in self.collision:
                continue
            if body_b_id in self.env.obstacle_collision_set:
                self.collision_obstacle = True
            if body_b_id in self.env.robot_collision_set:
                self.collision_vehicle = True
            if self.collision_obstacle or self.collision_vehicle:
                break
