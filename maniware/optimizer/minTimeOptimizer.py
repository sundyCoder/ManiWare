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

class MinTimeOptimizer(Optimizer):
    def __init__(self, op_name="MinTime"):
        self.op_name = op_name

    def optimize(self, robots, tasks):
        m, n = len(robots), len(tasks)
        result = set()
        if m == 0 or n == 0:
            return result
        Map = np.zeros((m, n))
        for j in range(n):
            for i in range(m):
                Map[i][j] = self.get_cost(robots[i].chassis.get_position(), tasks[j].get_position())
        while np.min(Map) < 1000.:
            a, b = np.unravel_index(np.argmin(Map), Map.shape)
            robots[a].thing = tasks[b]
            result.add(tasks[b])
            for j in range(n):
                for i in range(m):
                    if j == b or i == a:
                        Map[i][j] = np.inf
        return result  # a set, things has been assigned

    def get_cost(self, pos1, pos2):
        return np.linalg.norm(np.array(pos1) - np.array(pos2))