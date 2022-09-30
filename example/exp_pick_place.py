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

import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from maniware.application.pickPlaceApp import PickAndPlaceApp
from maniware.scheduler import PickPlaceScheduler
import time

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
    if timestep > .04 or i%(int(1/(24*timestep))) == 0:
        elapsed = time.time() - start_time
        if elapsed < (i*timestep):
            time.sleep(timestep*i - elapsed)


def get_action(robots, ROBOT, thingset, THING):
    # ROBOT = [[1, position_y, robot.action, robot.reward] ... ]
    # THING = [[2, position[0], position[1]] ... ]
    # robots : all robot in env
    # thingset:  avaliable_things
    ##### assign task for every robot
    result = [None for i in range(len(robots))]
    for i in range(len(robots)):
        dis = 500.
        things = list(thingset)
        for j in range(len(things)):
            if ROBOT[i][0] == THING[j][0] and (THING[j][2] - ROBOT[i][1]) >= -0.1 and (THING[j][2] - ROBOT[i][1]) <= 0.4:
                if (THING[j][2] - ROBOT[i][1]) <= dis:
                    dis = (THING[j][2] - ROBOT[i][1])
                    result[i] = things[j]
    return result


def main():
    env = PickAndPlaceApp(robot_config=[{'Suction': 1}, {'Gripper': 1}], thing_config=[{'cube':10}, {'cylinder':10}])
    episode = 10
    for k in range(episode):
        # margin: the distance between thing and next thing
        obs = env.reset(speed = -0.2, cube_num = 10, cylinder_num = 10, margin=1.)
        scheduler = PickPlaceScheduler(env.robots, env.available_thing_ids_set)
        start = time.time()
        step, done, R = 0, False, 0
        while not done:
            # action = get_action(env.robots, obs[0], env.available_thing_ids_set, obs[1])
            # obs, reward, done, info = env.step(action)
            scheduler.allocate()
            obs, reward, done, info = env.step()
            R += reward
            sync(step, start, env.TIMESTEP)
            step += 1
    env.close()


if __name__ == "__main__":
    main()

