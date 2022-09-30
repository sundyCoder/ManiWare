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
from .taskScheduler import TaskScheduler


class DynamicConfigureScheduler(TaskScheduler):
    def __init__(self, robots, tasks):
        self.T1 = set()
        self.T2 = set()
        self.robot = robots[0]
        for thing in tasks:
            if thing.rtype == 'cube':
                self.T1.add(thing)
            else:
                self.T2.add(thing)
        
    def min_distance_method(self, robot, tasks):
        # robot: robot information
        # tasks: a list of tasks to be allocated
        n = len(tasks)
        result = set()
        b = 0
        if n == 0:
            return result
        minDis = np.inf
        for j in range(n):
            dis = self.cal_distance(robot.ee.get_position(), tasks[j].get_position())
            if dis <= minDis:
                minDis = dis
                b = j
        robot.thing = tasks[b]
        result.add(tasks[b])
        return result  # a set, tasks has been assigned
        
    def cal_distance(self, pos1, pos2):
        return np.linalg.norm(np.array(pos1) - np.array(pos2))
        
    def allocate(self):
        task1 = list(self.T1)
        task2 = list(self.T2)
        set1 = set()
        set2 = set()
        if self.robot.rtype == 'Suction':
            if self.robot.action == 'idle':
                set1 = self.min_distance_method(self.robot, task1)
        else:
            if self.robot.action == 'idle':
                set2 = self.min_distance_method(self.robot, task2)
        self.T1 = self.T1 - set1
        self.T2 = self.T2 - set2
    
