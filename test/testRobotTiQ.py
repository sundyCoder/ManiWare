import os
import time
import pdb
import math
import pybullet as p
import pybullet_data
from attrdict import AttrDict
from collections import namedtuple

robotUrdfPath = '../maniware/assets/ur5_defi/gripper/robotiq_2f_85.urdf'
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# define world
p.setGravity(0, 0, -10)
planeID = p.loadURDF("plane.urdf")
controlJoints = ["robotiq_2f_85_right_driver_joint",
                "robotiq_2f_85_right_follower_joint",
                "robotiq_2f_85_right_spring_link_joint",
                "robotiq_2f_85_left_driver_joint",
                "robotiq_2f_85_left_follower_joint",
                "robotiq_2f_85_left_spring_link_joint"]

robotStartPos = [0, 0, 0.15]
robotStartOrn = p.getQuaternionFromEuler([0, 1.57, 0])
robotID = p.loadURDF(robotUrdfPath, robotStartPos, robotStartOrn, flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

jointTypeList = ["REVOLUTE", "PRISMATIC", "SPHERICAL", "PLANAR", "FIXED"]
numJoints = p.getNumJoints(robotID)
jointInfo = namedtuple("jointInfo", ["id", "name", "type", "lowerLimit", "upperLimit", "maxForce", "maxVelocity"])

joints = AttrDict()

# get jointInfo and index of dummy_center_indicator_link
for i in range(numJoints):
    info = p.getJointInfo(robotID, i)
    jointID = info[0]
    jointName = info[1].decode("utf-8")
    jointType = jointTypeList[info[2]]
    jointLowerLimit = info[8]
    jointUpperLimit = info[9]
    jointMaxForce = info[10]
    jointMaxVelocity = info[11]
    singleInfo = jointInfo(jointID, jointName, jointType, jointLowerLimit, jointUpperLimit, jointMaxForce, jointMaxVelocity)
    joints[singleInfo.name] = singleInfo
    # register index of dummy center link


gripper_main_control_joint_name = "robotiq_2f_85_right_driver_joint"
mimic_joint_name = ["robotiq_2f_85_right_follower_joint",
                    "robotiq_2f_85_right_spring_link_joint",
                    "robotiq_2f_85_left_driver_joint",
                    "robotiq_2f_85_left_follower_joint",
                    "robotiq_2f_85_left_spring_link_joint"]
mimic_multiplier = [-1, 1, 1, -1, 1]

gripper_opening_length_control = p.addUserDebugParameter("gripper_opening_length", 0, 0.085, 0.085)

while True:
    # gripper control
    gripper_opening_length = p.readUserDebugParameter(gripper_opening_length_control)
    gripper_opening_angle = 0.715 - math.asin((gripper_opening_length - 0.010) / 0.1143)  # angle calculation

    p.setJointMotorControl2(robotID,
                            joints[gripper_main_control_joint_name].id,
                            p.POSITION_CONTROL,
                            targetPosition=gripper_opening_angle,
                            force=joints[gripper_main_control_joint_name].maxForce,
                            maxVelocity=joints[gripper_main_control_joint_name].maxVelocity)
    for i in range(len(mimic_joint_name)):
        joint = joints[mimic_joint_name[i]]
        p.setJointMotorControl2(robotID, joint.id, p.POSITION_CONTROL,
                                targetPosition=gripper_opening_angle * mimic_multiplier[i],
                                force=joint.maxForce,
                                maxVelocity=joint.maxVelocity)

    p.stepSimulation()
