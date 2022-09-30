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

import os
import numpy as np
import pybullet as p
import time
from maniware.mani_level.component.endEffectors import Suction, Robotiq85
from maniware.mani_level.utils import (
    control_joints,
    set_joint_positions,
    get_joint_positions,
    get_link_pose,
    inverse_kinematics,
    forward_kinematics
)


def distance_XY(position1, position2):
    return np.sqrt((position1[0]-position2[0])**2 + (position1[1]-position2[1])**2)

def distance(position1, position2):
    return np.sqrt((position1[0] - position2[0]) ** 2 + (position1[1] - position2[1]) ** 2 + (position1[2] - position2[2]) ** 2)



class UR5_new():
    joint_epsilon = 0.01
    joints_count = 6
    next_available_color = 0
    workspace_radius = 0.85
    colors = [
        [0.4, 0, 0],
        [0, 0, 0.4],
        [0, 0.4, 0.4],
        [0.4, 0, 0.4],
        [0.4, 0.4, 0],
        [0, 0, 0],
    ]
    LINK_COUNT = 10

    GROUPS = {
        'arm': ["shoulder_pan_joint",
                "shoulder_lift_joint",
                "elbow_joint",
                "wrist_1_joint",
                "wrist_2_joint",
                "wrist_3_joint"],
        'gripper': None
    }

    GROUP_INDEX = {
        'arm': [0, 1, 2, 3, 4, 5],
        'gripper': None
    }


    LOWER_LIMITS = [-np.pi, -2.3562, -np.pi, -17, -17, -17]
    UPPER_LIMITS = [np.pi, -0.523, np.pi, 17, 17, 17]
    JOINT_RANGES = [2 * np.pi, 1.8332, 2 * np.pi, 34, 34, 34]
    MAX_VELOCITY = [3.15, 3.15, 3.15, 3.2, 3.2, 3.2]
    MAX_FORCE = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

    HOME = [0, 0, 0, 0, 0, 0]
    UP = [0, -1.5707, 0, -1.5707, 0, 0]
    RESET = [0, -1, 1, 0.5, 1, 0]
    EEF_LINK_INDEX = 7

    def __init__(self, env, pose, fix, rtype, height, velocity=0.8, enabled=True, acceleration=1.0, training=True):

        self.velocity = velocity
        self.acceleration = acceleration
        self.pose = pose
        self.env = env
        self.height = height
        self.rtype = rtype
        self.color = self.colors[self.next_available_color]
        self.next_available_color = (self.next_available_color + 1) % len(self.colors)

        self.enabled = enabled
        self.subtarget_joint_actions = False
        self.grab_finished = True
        self.ready = True

        self.action = 'idle'
        self.last_action = 'idle'
        self.waitNum = 0
        self.ik_flag = False
        self.flag = 0

        self.pick_pose = None
        self.place_pose = None
        self.place_position = None
        self.place_position2 = None

        self.thing = None
        
        self.offset_x = 0.
        self.numWaiting = 0
        '''
        if training:
            self.id = p.loadURDF('../../assets/ur5/ur5_training.urdf',
                                      self.pose[0],
                                      self.pose[1],
                                      flags=p.URDF_USE_SELF_COLLISION)
            self.ee = None
            p.changeVisualShape(
                self.id,
                self.EEF_LINK_INDEX,
                textureUniqueId=-1,
                rgbaColor=(
                    self.color[0],
                    self.color[1],
                    self.color[2], 0.5))
        else:
        '''
        ur5_path = os.path.join(os.path.dirname(__file__), "../../assets/ur5/ur5.urdf")
        self.id = None
        if self.rtype == 'Suction':
            self.id = p.loadURDF(ur5_path, self.pose[0], self.pose[1], flags=p.URDF_USE_SELF_COLLISION, useFixedBase = fix)
            self.ee = Suction(self.env, self, self.env.things)
            self.offset_x = -0.3
        else:
            self.id = p.loadURDF(ur5_path, self.pose[0], self.pose[1], flags=p.URDF_USE_SELF_COLLISION, useFixedBase = fix)
            self.ee = Robotiq85(self.env, self, 7, self.env.things)
            self.offset_x = 0.3


        robot_joint_info = [p.getJointInfo(self.id, i, physicsClientId=self.env.client) for i in range(p.getNumJoints(self.id))]
        self._robot_joint_indices = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_lower_limits = [x[8] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self._robot_joint_upper_limits = [x[9] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
        self.controled_joints = self._robot_joint_indices

        self.home_config = [0., - 2 * np.pi / 3, 2* np.pi / 3, -np.pi / 2, -np.pi / 2, np.pi / 2] # [0., - np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, np.pi / 2]
        self.place_config = [np.pi, - 2 * np.pi / 3, 2* np.pi / 3, -np.pi / 2, -np.pi / 2, np.pi / 2]
        self.target_joint_values = self.home_config


    def change_ee(self, rtype):
        if self.rtype == rtype:
            pass
        else:
            a = time.time()
            self.ee.remove()
            self.rtype = rtype
            if self.rtype == 'Suction':
                self.ee = Suction(self.env, self, self.env.things)
                self.height = 0.15
            else:
                self.ee = Robotiq85(self.env, self, 7, self.env.things)
                self.height = 0.1

            time_step = 0
            while time_step < 2200:
                p.stepSimulation(physicsClientId=self.env.client)
                time_step += 1
            self.ready = True



    def update_closest_points(self):
        pass

    def check_collision(self, collision_distance):
        # collision with other

        # self collision
        pass
        
    def step_discrete(self):
        if self.action == 'idle' and self.thing:
            self.pick_pose = ((self.thing.get_position()[0], self.pose[0][1], self.thing.get_position()[2] + self.height), self.ee.orientation)
            # self.target_joint_values = self.inverse_kinematics(self.pick_pose[0])
            self.target_joint_values = [-0.0918, -1.681, 2.065, -1.606, -1.585, np.pi/2]
            self.working = True
            self.action = 'goToGrab'
            self.last_action = 'idle'
        elif self.action == 'goToGrab':
            self._move_jointsss(self.target_joint_values, self.flag, 0.05)
            if self.ik_flag == False:
                """
                robot_joint_info = [p.getJointInfo(self.id, i, physicsClientId=self.env.client) for i in range(p.getNumJoints(self.id))]
                index = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
                temp = []
                for item in index:
                    temp.append(p.getJointState(self.id, item)[0])
                print(self.rtype, temp)
                """
                self.place_pose = ((self.pose[0][0] + self.offset_x, self.pose[0][1], self.pose[0][2] + 0.5), self.ee.orientation)
                # self.target_joint_values = self.inverse_kinematics(self.place_pose[0])
                self.target_joint_values = self.place_config
                self.action = 'grab'
                self.last_action = 'goToGrab'
        elif self.action == 'grab':
            # judge whether it can grab thing, and process
            #if distance_XY(self.thing.get_position(), self.ee.get_position()) <= 0.2:
            #points = p.getContactPoints(bodyA=self.ee.id_body, linkIndexA=0)
            #if points:
            #    for point in points:
            #        obj_id, contact_link = point[2], point[4]
            #    if obj_id == self.thing.id:
            #        self.thing.speed = None

            if self.rtype == 'Suction':
                if self.ee.detect_collision_with_thing(self.thing) or distance_XY(self.thing.get_position(), self.ee.get_position()) <= 0.35:
                    p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.], physicsClientId=self.env.client)
                    self.ee.grab_thing(self.thing)
                    self.thing.finished = True
                if self.ee.check_grasp():
                    self.action = 'backToPlace'
                    self.last_action = 'grab'
            elif self.rtype == 'Gripper':
                self.ee.still()
                if distance_XY(self.thing.get_position(), self.ee.get_position()) <= 0.3:
                    p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.],
                                        physicsClientId=self.env.client)
                    self.ee.grab_thing(self.thing)
                    self.thing.finished = True
                    self.action = 'closeGripper'
                    self.last_action = 'grab'
        elif self.action == 'closeGripper':
            self.ee.close_gripper()
            self.action = 'backToPlace'
            self.last_action = 'closeGripper'
            if self.ee.open_close_flag == False:
                self.action = 'backToPlace'
                self.last_action = 'closeGripper'
        elif self.action == 'backToPlace':
            # print(self.action)
            # go to the place pose
            # self._move(self.place_pose[0], self.place_pose[1], self.flag, self.ur_speed)
            self._move_jointsss(self.target_joint_values, self.flag, 0.05)
            if self.ik_flag == False:
                """
                robot_joint_info = [p.getJointInfo(self.id, i, physicsClientId=self.env.client) for i in range(p.getNumJoints(self.id))]
                index = [x[0] for x in robot_joint_info if x[2] == p.JOINT_REVOLUTE]
                temp = []
                for item in index:
                    temp.append(p.getJointState(self.id, item)[0])
                print(self.type, temp)
                """
                self.action = 'place'
                self.last_action = 'backToPlace'
                if self.rtype == 'Gripper':
                    self.ee.still()
                    self.action = 'openGripper'
                    self.last_action = 'backToPlace'
        elif self.action == 'openGripper':
            self.ee.open_gripper()
            self.action = 'place'
            self.last_action = 'openGripper'
            if self.ee.open_close_flag == False:
                self.action = 'place'
                self.last_action = 'openGripper'

        elif self.action == 'place':
            # release the ee
            if self.rtype == 'Suction':
                self._reward = self.ee.throw_thing()
            elif self.rtype == 'Gripper':
                self.ee.still()
                self._reward = self.ee.throw_thing()
            p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.],
                                physicsClientId=self.env.client)
            self.action = 'reset'
            self.last_action = 'place'
            self.pick_pose = None
            self.working = False
            self.env.available_thing_ids_set.remove(self.thing)
            self.env.put_grasped_thing_on_conveyor.add(self.thing)
            self.thing = None
        
        elif self.action == 'reset':
            # self.move_joint_discrete(self.home_config, self.flag, self.ur_speed)
            self._move_jointsss(self.home_config, self.flag, 0.05)
            if self.ik_flag == False:
                self.action = 'waiting'
                self.last_action = 'reset'
        
        elif self.action == 'waiting':
            self.numWaiting += 1
            if self.numWaiting >= 10:  # 3000
                self.numWaiting = 0
                self.action = 'idle'
                self.last_action = 'place'

    def pick_and_place_FSM(self):
        if self.action == 'idle':
            if self.thing:
                self.grab_finished = False
                self.target_joint_values = self.inverse_kinematics((self.thing.get_position()[0], self.thing.get_position()[1], self.thing.get_position()[2] + self.height))
                # self.thing = thing
                self.action = 'goToGrab'
                self.last_action = 'idle'
                self.working = False
            else:
                self.step()

        elif self.action == 'goToGrab':
            # move to the pose that ur can grab something
            # self.step()
            # if distance(self.thing.get_position(), self.ee.get_position()) <= 0.4:
            self._move_jointsss(self.target_joint_values, self.flag, 0.05)
            if self.ik_flag == False:
                self.action = 'grab'
                self.last_action = 'goToGrab'

        elif self.action == 'grab':
            if distance(self.thing.get_position(), self.ee.get_position()) <= 0.4:
                self.ee.grab_thing(self.thing)
                self.grab_finished = True
                self.action = 'toBack'
                self.last_action = 'grab'

        elif self.action == 'toBack':
            if not self.place_position is None:
                self.target_joint_values = self.inverse_kinematics(self.place_position)
                self.action = 'back'
                self.last_action = 'toBack'
            if not self.place_position2 is None:
                self.target_joint_values = self.inverse_kinematics(self.place_position2)
                self.action = 'back'
                self.last_action = 'toBack'

        elif self.action == 'back':
            self._move_jointsss(self.target_joint_values, self.flag, 0.05)
            if self.ik_flag == False:
                self.action = 'place'
                self.last_action = 'back'

        elif self.action == 'place':
            # release the ee
            if self.rtype == 'Suction':
                self._reward = self.ee.throw_thing()
            elif self.rtype == 'Gripper':
                self.ee.still()
                self._reward = self.ee.throw_thing()
            p.resetBaseVelocity(self.thing.id, linearVelocity=[0., 0., 0.], angularVelocity=[0., 0., 0.],
                                physicsClientId=self.env.client)
            p.resetBasePositionAndOrientation(self.thing.id, [-10, -10, 5], [0., 0., 0., 1.], physicsClientId=self.env.client)
            self.action = 'reset'
            self.last_action = 'place'
            self.pick_pose = None


            self.target_joint_values = self.home_config

        elif self.action == 'reset':
            self._move_jointsss(self.target_joint_values, self.flag, 0.05)
            if self.ik_flag == False:
                self.action = 'waiting'
                self.last_action = 'reset'


        elif self.action == 'waiting':
            self.waitNum += 1
            if self.waitNum >= 480:
                self.waitNum = 0
                self.action = 'idle'
                self.last_action = 'waiting'
                self.env.available_thing_ids_set.remove(self.thing)
                self.env.removed_thing_ids_set.add(self.thing)
                self.place_position = None
                self.working = False
                self.thing = None



    def compute_next_subtarget_joints(self):
        current_joints = self.get_arm_joint_values()
        if type(self.target_joint_values) != np.ndarray:
            self.target_joint_values = np.array(self.target_joint_values)
        subtarget_joint_values = self.target_joint_values - current_joints
        dt = 1. / self.env.hz
        dj = dt * self.velocity
        max_relative_joint = max(abs(subtarget_joint_values))
        if max_relative_joint < dj: # if it can reach in 1 step
            return subtarget_joint_values
        subtarget_joint_values = dj * subtarget_joint_values / max_relative_joint
        return subtarget_joint_values + current_joints

    def disable(self):
        self.enabled = False
        # self.set_pose(self.init_pose, [0., 0., 0., 1.])
        self.reset()
        self.step()

    def enable(self):
        self.enabled = True

    def step(self):
        if self.ee is not None:
            self.ee.still()
        if self.subtarget_joint_actions:
            control_joints(self.id, self.GROUP_INDEX['arm'], self.compute_next_subtarget_joints(), velocity=self.velocity, acceleration=self.acceleration)
        else:
            control_joints(self.id, self.GROUP_INDEX['arm'], self.target_joint_values, velocity=self.velocity,
                               acceleration=self.acceleration)

    def _move_jointsss(self, joints, flag, speed=0.01):
        if self.ik_flag == False:
            self.ik_flag = True
        self.move_joint_discrete(joints, flag, speed)

    def move_joint_discrete(self, target_joint, flag, speed=0.01):
        # flag:   record how many step it has passed
        # compute one step control
        current_joint = [p.getJointState(self.id, i, physicsClientId=self.env.client)[0] for i in self.controled_joints]
        current_joint = np.array(current_joint)
        different = target_joint - current_joint
        if all(np.abs(different) < 0.2):
            self.target_joint = None
            self.ik_flag = False
            flag = 0

        # Move with constant velocity
        norm = np.linalg.norm(different)
        v = different / norm if norm > 0 else 0
        step_joint = current_joint + v * speed
        gains = np.ones(len(self.controled_joints))
        p.setJointMotorControlArray(
            bodyIndex=self.id,
            jointIndices=self.controled_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=step_joint,
            positionGains=gains, physicsClientId=self.env.client)
        flag += 1

    def solve_IK(self, goal_pos, Orientation):
        targetPositionsJoints = p.calculateInverseKinematics(self.id, 7, goal_pos, targetOrientation=Orientation,
                                                             lowerLimits=[-2 * np.pi, -3, -np.pi, -2 * np.pi,
                                                                          -2 * np.pi, -2 * np.pi],
                                                             upperLimits=[2 * np.pi, -0.5, np.pi, 2 * np.pi, 2 * np.pi,
                                                                          2 * np.pi],
                                                             jointRanges=[4 * np.pi, 2.5, 2 * np.pi, 4 * np.pi,
                                                                          4 * np.pi, 4 * np.pi],  # * 6,
                                                             restPoses=np.float32(np.array(
                                                                 [-1, -0.5, 0.5, -0.5, -0.5, 0]) * np.pi).tolist(),
                                                             maxNumIterations=100,
                                                             residualThreshold=1e-5, physicsClientId=self.env.client)
        joints = np.float32(targetPositionsJoints)
        joints[2:] = (joints[2:] + np.pi) % (2 * np.pi) - np.pi
        return joints

    def get_pose(self):
        return p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)

    def set_pose(self, pose):
        self.pose = pose
        p.resetBasePositionAndOrientation(self.id, self.pose[0], self.pose[1], physicsClientId=self.env.client)
        if self.ee is not None:
            self.ee.update_ee_pose()

    def global_to_ur5_frame(self, position, rotation=None):
        # relative pos and rot in ur5_base frame
        self_pos, self_rot = p.getBasePositionAndOrientation(self.id, physicsClientId=self.env.client)
        invert_self_pos, invert_self_rot = p.invertTransform(
            self_pos, self_rot)
        ur5_frame_pos, ur5_frame_rot = p.multiplyTransforms(
            invert_self_pos, invert_self_rot,
            position, invert_self_rot if rotation is None else rotation
        )
        return ur5_frame_pos, ur5_frame_rot

    def get_link_global_positions(self):
        linkstates = [p.getLinkState(self.id, link_id, computeForwardKinematics=True, physicsClientId=self.env.client) for link_id in range(UR5.LINK_COUNT)]
        link_world_positions = [world_pos for world_pos, world_rot, _, _, _, _, in linkstates]
        return link_world_positions

    def get_arm_joint_values(self):
        return np.array(get_joint_positions(self.id, self.GROUP_INDEX['arm']))

    def reset(self):
        self.set_arm_joints(self.home_config)

    def get_end_effector_pose(self, link=None):
        link = link if link is not None else self.EEF_LINK_INDEX
        return get_link_pose(self.id, link)


    def set_target_end_eff_pos(self, pos):
        self.set_arm_joints(self.inverse_kinematics(position=pos))

    def inverse_kinematics(self, position, orientation=None):
        targetPositionsJoints = inverse_kinematics(self.id, self.EEF_LINK_INDEX, position, self.LOWER_LIMITS, self.UPPER_LIMITS, self.JOINT_RANGES, orientation)
        joints = np.float32(targetPositionsJoints)
        joints[1:-1] = (joints[1:-1] + np.pi) % (2 * np.pi) - np.pi
        return joints

    def forward_kinematics(self, joint_values):
        return forward_kinematics(self.id, self.GROUP_INDEX['arm'], joint_values, self.EEF_LINK_INDEX)

    def control_arm_joints(self, joint_values, velocity=None):
        velocity = self.velocity if velocity is None else velocity
        self.target_joint_values = joint_values
        if not self.subtarget_joint_actions:
            control_joints(self.id, self.GROUP_INDEX['arm'], self.target_joint_values, velocity=velocity, acceleration=self.acceleration)


    def set_arm_joints(self, joint_values):
        set_joint_positions(self.id, self.GROUP_INDEX['arm'], joint_values)
        self.control_arm_joints(joint_values=joint_values)
        # if self.ee is not None:
        #     self.ee.update_ee_pose()
