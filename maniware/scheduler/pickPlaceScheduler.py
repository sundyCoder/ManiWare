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

from .taskScheduler import TaskScheduler
from maniware.optimizer.minSumOptimizer import MinSumOptimizer


class PickPlaceScheduler(TaskScheduler):
    def __init__(self, robots, tasks):
        self.R1 = set()
        self.R2 = set()
        self.T1 = set()
        self.T2 = set()
        self.optimizer = MinSumOptimizer()
        for robot in robots:
            if robot.rtype == 'Suction':
                self.R1.add(robot)
            else:
                self.R2.add(robot)
        for thing in tasks:
            if thing.rtype == 'cube':
                self.T1.add(thing)
            else:
                self.T2.add(thing)
        
    def allocate(self):
        idleR1, idleR2 = [], []
        task1, task2 = [], []
        for item in self.R1:
            if item.thing == None:
                idleR1.append(item)
        for item in self.R2:
            if item.thing == None:
                idleR2.append(item)
        for item in self.T1:
            if item.finished == False:
                task1.append(item)
        for item in self.T2:
            if item.finished == False:
                task2.append(item)
                
        self.optimizer.optimize(idleR1, task1)
        self.optimizer.optimize(idleR2, task2)