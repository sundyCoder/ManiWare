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
from .optimizer import Optimizer


class MinSumOptimizer(Optimizer):
    def __init__(self, op_type="MinTime"):
        self.op_type = op_type

    def optimize(self, robots, things):
        # robots: a list, robot.thing == None
        # things: a list, things not allocated
        m = len(robots)
        n = len(things)
        result = set()
        if m == 0 or n == 0:
            return result
        Map = np.zeros((m, n))
        for i in range(m):
            dis = 500.
            temp = None
            for j in range(n):
                Map[i][j] = self.get_cost(robots[i].position, things[j].get_position())
                if Map[i][j] <= dis and Map[i][j] >= -0.1 and Map[i][j] <= 0.4:
                    dis = Map[i][j]
                    temp = things[j]
            robots[i].thing = temp

    def get_cost(self, pos1, pos2):
        return pos2[1] - pos1[1]
