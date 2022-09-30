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

import time
import os, sys

currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from maniware.application.collectionApp import CollectApp
from maniware.scheduler.traPlanScheduler import TraPlanScheduler


def sync(i, start_time, timestep):
    """Syncs the stepped simulation with the wall-clock.
    Function `sync` calls time.sleep() to pause a for-loop
    running faster than the expected timestep.
    Parameters
    ----------
    i : int
        Current simulation iteration.
    start_time : timestamp
        Timestamp of the simulation start.
    timestep : float
        Desired, wall-clock step of the simulation's rendering.
    """
    if timestep > .04 or i % (int(1 / (24 * timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i * timestep):
            time.sleep(timestep * i - elapsed)


def main():
    env = CollectApp(robot_config=[{'Suction': 4}, {'Gripper': 4}], thing_config=[{'cube': 10}, {'cylinder': 10}])
    episode = 3
    for k in range(episode):
        obs = env.reset(cube_num=10, cylinder_num=10)
        # R1, R2, T1, T2 = init(env.robots, env.available_thing_ids_set)
        scheduler = TraPlanScheduler(env.robots, env.available_thing_ids_set)
        start = time.time()

        step, done, R = 0, False, 0
        while not done:
            # time.sleep(100)
            # T1, T2 = taskAs(T1, T2, R1, R2)
            scheduler.allocate()
            obs, reward, done, info = env.step()
            R += reward
            sync(step, start, env.TIMESTEP)
            step += 1
    env.close()


if __name__ == "__main__":
    main()


