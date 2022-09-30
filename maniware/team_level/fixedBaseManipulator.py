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

from .executor import Executor
from maniware.mani_level.component.UR5 import UR5_new


class FixedBaseManipulator(Executor):
    def __init__(self, env, pose, height, ee_type = 'Suction'):
        self.env = env
               
        self.init_pose = pose
        self.position = self.init_pose[0]
        self.height = height
        
        self.ee_type = ee_type
        self.rtype = ee_type

        self.ee = None
        self.ur5 = None
        
        self.state = None
        self.reward = 0
        self._reward = 0
        
        self.action = None    # idle, goToGrab, grab, goToplace, place, idle
        self.last_action = None      # the action state that robot has just finished
        self.thing = None

        self.pick_pose = None   # the pose before it grab the task
        self.place_pose = None
        self.waitNum = 0

        self._createBody()
        
    def _createBody(self):
        self.ur5 = UR5_new(self.env, self.init_pose, 1, self.ee_type, self.height)
        
    def reset(self):
        self.ur5.reset()
        self.ur5.ee.release()

        self.reward = 0  # Cumulative returned rewards.
        self.action = 'idle'   # move, idle, grab
        self.working = False
        
    def step(self):
        self.ur5.thing = self.thing
        self.ur5.step_discrete()
        if self.ur5.thing == None:
            self.thing = None
        
    def state(self):
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
        
    