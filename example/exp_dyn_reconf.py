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

from maniware.application.dynReconfApp import ReconfApp
from maniware.scheduler import DynamicConfigureScheduler



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
    env = ReconfApp(robot_config=[{'Suction': 0}, {'Gripper': 1}], thing_config=[{'cube': 2}, {'cylinder': 2}])
    episode = 3
    for k in range(episode):
        obs = env.reset(cube_num=2, cylinder_num=2)
        # robot, T1, T2 = init(env.robots, env.available_thing_ids_set)
        scheduler = DynamicConfigureScheduler(env.robots, env.available_thing_ids_set)
        scheduler.robot.change_ee('Suction')
        start = time.time()

        step, done, R = 0, False, 0
        while not done:
            # time.sleep(100)
            # action = get_action(env.robots, obs[0], env.available_thing_ids_set, obs[1])
            if len(env.available_thing_ids_set) == 2 and scheduler.robot.action == 'idle' and scheduler.robot.rtype=='Suction':
                scheduler.robot.ready = False
                scheduler.robot.change_ee('Gripper')
            # T1, T2 = taskAs(robot, T1, T2)
            scheduler.allocate()
            obs, reward, done, info = env.step()
            R += reward
            sync(step, start, env.TIMESTEP)
            step += 1
    env.close()


if __name__ == "__main__":
    main()


