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
import pybullet_data
import os, sys
currentdir = os.path.dirname(os.path.realpath(__file__))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)

from maniware.team_level import FixedBaseManipulator
from maniware.team_level import ManiTask


class PickAndPlaceApp:
    '''
    robot_config = {'type1': robot1_num, 'type2': robot2_num}
    box_config = {'cube': cube_num, 'cylinder': cylinder_num}
    num_box:   max number of each tye boxes
    '''
    def __init__(self, display=True, hz=240, robot_config=None, thing_config=None, env_name='UR_Conveyor'):
        self.robot_config = robot_config
        self.thing_config = thing_config
        self.env_name = env_name
        self.hz = hz
        self.TIMESTEP = 1. / self.hz

        self.conveyor_speed = 0
        self.margin = 0
        self.last_thing_cube = None
        self.last_thing_cylinder = None

        if self.robot_config is None:
            self.robot_config = [{'Suction': 2}, {'Gripper': 2}]
        self.num_robots = sum(sum(g.values()) for g in self.robot_config)
        self.robot_group_types = [next(iter(g.keys())) for g in self.robot_config]

        if self.thing_config is None:
            self.thing_config = [{'cube': 2}, {'cylinder': 2}]
        self.num_things = sum(sum(g.values()) for g in self.thing_config)
        self.thing_group_types = [next(iter(g.keys())) for g in self.thing_config]

        self.robots = []      # the robot object
        self.robot_groups = [[] for _ in range(len(self.robot_config))]  # [[all object of robot1], [all object of robot2]]

        self.things = []
        self.thing_groups = [[] for _ in range(len(self.thing_config))]

        self.available_thing_ids_set = None     # things took part in the task
        self.removed_thing_ids_set = None       # things not in task

        self.put_leaked_thing_on_conveyor = None
        self.put_grasped_thing_on_conveyor = None

        self.client = None

        self.display = display
        if self.display:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client)
        p.resetSimulation(physicsClientId=self.client)
        p.setTimeStep(1. / hz, physicsClientId=self.client)
        p.setGravity(0., 0., -10, physicsClientId=self.client)  # -9.8
        self._create_env()


    def _create_env(self):
        planeID = p.loadURDF("plane.urdf")

        conveyor_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.4, 40, 0.2), physicsClientId=self.client)
        conveyor_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.4, 40, 0.2),
                                                       rgbaColor=[0.4, 0.4, 0.4, 1.0], physicsClientId=self.client)
        self.conveyor_id = p.createMultiBody(0, conveyor_collision_shape_id, conveyor_visual_shape_id, (0, 0, 0), physicsClientId=self.client)

        platform_collision_id_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), physicsClientId=self.client)
        platform_visual_id_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), rgbaColor=[1., 1., 1., 1.0], physicsClientId=self.client)
        self.platform1_id = p.createMultiBody(0, platform_collision_id_1, platform_visual_id_1, (-0.6, 0, 0), physicsClientId=self.client)

        platform_collision_id_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), physicsClientId=self.client)
        platform_visual_id_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 24, 0.2), rgbaColor=[1., 1., 1., 1.0], physicsClientId=self.client)
        self.platform2_id = p.createMultiBody(0, platform_collision_id_2, platform_visual_id_2, (0.6, 0, 0), physicsClientId=self.client)

        self.thing_groups = [[] for _ in range(len(self.thing_config))]
        for thing_group_index, t in enumerate(self.thing_config):
            thing_type, count = next(iter(t.items()))
            # add 5 cube and 5 cylinder in the env, people can't see it. To decrease the time that reset() will speed.
            for n in range(20):
                if thing_type == 'cylinder':
                    Y = 1
                elif thing_type == 'cube':
                    Y = 2
                thing = ManiTask(self, [4, Y, 0.3 * n], thing_type)  # load things
                self.things.append(thing)
                self.thing_groups[thing_group_index].append(thing)

        for robot_group_index, g in enumerate(self.robot_config):
            robot_type, count = next(iter(g.items()))
            for kk in range(count):
                O = None
                height = 0.
                if robot_type == "Suction":
                    X = -0.6
                    x = -1.5
                    O = (0., 0., 0.)
                    height = 0.4
                elif robot_type == "Gripper":
                    X = 0.6
                    x = 1.5
                    O = (0., 0., np.pi)
                    height = 0.4
                robot = FixedBaseManipulator(self, ([X, 0.6 - kk * 1.2, 0.2], np.asarray(p.getQuaternionFromEuler(O))), height, robot_type)  # set the pose of ur
                self.robots.append(robot)
                self.robot_groups[robot_group_index].append(robot)

        self.available_thing_ids_set = set()  # a set that include the task box
        self.removed_thing_ids_set = set(self.things)  # not in task

    def add_object(self, position, thing_type):
        thing = ManiTask(self, position, thing_type)
        self.things.append(thing)
        self.thing_groups[thing_type].append(thing)
        return thing

    def seed(self, seed=None):
        self._random = np.random.RandomState(seed)
        return seed

    def reset(self, speed, cube_num=1, cylinder_num=1, margin=0.5):
        # init
        self.available_thing_ids_set = set()  # a set that include the task box
        self.removed_thing_ids_set = set(self.things)  # not in task
        
        self.put_leaked_thing_on_conveyor = set()
        self.put_grasped_thing_on_conveyor = set()

        self.conveyor_speed = speed
        self.margin = margin
        for thing in self.things:
            thing.finished = False
            thing.speed = self.conveyor_speed
        for i in range(cube_num):
            self.available_thing_ids_set.add(self.thing_groups[0][i])
            if self.thing_groups[0][i] in self.removed_thing_ids_set:
                self.removed_thing_ids_set.remove(self.thing_groups[0][i])
        for j in range(cylinder_num):
            self.available_thing_ids_set.add(self.thing_groups[1][j])
            if self.thing_groups[1][j] in self.removed_thing_ids_set:
                self.removed_thing_ids_set.remove(self.thing_groups[1][j])

        for robot in self.robots:
            robot.reset()
        
        self.put_things_on_conveyor(self.available_thing_ids_set, self.margin)
        p.setRealTimeSimulation(0, physicsClientId=self.client)
        obs = self._computeObs()
        return obs
            

    def _computeObs(self):
        # computer the obs, algorithm can use it to assign the task
        ROBOT = []
        for robot in self.robots:
            position_y = robot.position[1]
            if robot.rtype == 'Suction':
                ROBOT.append([1, position_y, robot.action, robot.reward])
            elif robot.rtype == 'Gripper':
                ROBOT.append([2, position_y, robot.action, robot.reward])
        THING = []
        for thing in self.available_thing_ids_set:
            position = thing.get_position()
            thing_id = thing.id
            if thing.rtype == 'cube':
                THING.append([1, thing_id, position[0], position[1]])
            elif thing.rtype == 'cylinder':
                THING.append([2, thing_id, position[0], position[1]])
        return [ROBOT, THING, self.conveyor_speed]


    def clearRobotAndEffector(self):
        if len(self.robots) > 0:
            for robot in self.robots:
                robot.remove()
                del robot
            self.robot_ids = []  # the model id
            self.robots = []  # the robot object
            self.robot_groups = [[] for _ in range(len(self.robot_config))]

    def put_things_on_conveyor(self, thing_sets, margin):
        # margin:  the distence between two things
        i = 0
        j = 0
        for thing in thing_sets:
            if thing.rtype == 'cube':
                thing.reset([[-0.2, 1.5 + i * margin, 0.125],[0., 0., 0., 1.]])
                i += 1
                self.last_thing_cube = thing
            else:
                thing.reset([[0.2, 1.5 + j * margin, 0.15],[0., 0., 0., 1.]])
                j += 1
                self.last_thing_cylinder = thing
            """
            if (i + j) % 2 == 0:
                #thing.reset(-0.3, 1.5 + i * margin)
                thing.reset(-0.2, j * margin - 0.5)
                j += 1
            else:
                #thing.reset(0.2, 1.5 + i * margin)
                thing.reset(0.2, i * margin - 0.5)
                i += 1
            #thing._move_on_conveyor()
            """

    def put_processed_things_on_conveyor(self):
        things_back = self.put_leaked_thing_on_conveyor + self.put_grasped_thing_on_conveyor

        for thing in self.put_grasped_thing_on_conveyor:
            self.put_grasped_thing_on_conveyor.remove(thing)
            self.available_thing_ids_set.add(thing)

        if len(things_back) >= 1:
            for thing in things_back:
                thing.finished = False
                if thing.rtype == 'cube':
                    thing.reset(-0.2, self.last_thing_cube.get_position()[1] + self.margin)  # x, y positions
                    self.last_thing_cube = thing
                elif thing.rtype == 'cylinder':
                    thing.reset(0.2, self.last_thing_cylinder.get_position()[1] + self.margin)
                    self.last_thing_cylinder = thing
            self.put_leaked_thing_on_conveyor = set()
            self.put_grasped_thing_on_conveyor = set()

    @property
    def info(self):
        info = {}
        return info

    def close(self):
        p.disconnect(physicsClientId=self.client)

    def step(self):
        for i in range(len(self.robots)):
            # control robot grab things
            self.robots[i].step()


        for thing in self.available_thing_ids_set:
            if thing.finished == False:
                thing._move()
            if thing.get_position()[1] < 0: # -3
                self.put_leaked_thing_on_conveyor.add(thing)
                thing.finished == True
        
        # self.put_processed_things_on_conveyor()

        p.stepSimulation(physicsClientId=self.client)
        reward, info = 0, {}

        done = False
        num_idle = 0
        for robot in self.robots:
            if robot.action == 'idle':
                num_idle += 1

        # if len(self.available_box_ids_set) <= 0 and num_idle == len(self.robots):
        if (len(self.put_grasped_thing_on_conveyor) + len(self.put_leaked_thing_on_conveyor)) >= len(self.available_thing_ids_set) and num_idle == len(self.robots):
            done = True

        # Add ground truth robot state into info.
        info.update(self.info)
        obs = self._computeObs()
        return obs, reward, done, info
