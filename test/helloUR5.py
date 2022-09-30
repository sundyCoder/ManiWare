import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
robot = p.loadURDF("../maniware/assets/ur/ur5e.urdf", [0, 0, 0], useFixedBase = 1)
#s = p.getNumJoints(robot)



p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

Orientation = p.getQuaternionFromEuler([math.pi, 0., 0])
goal_pos = [-0.7, 0.0, 0.3]
targetPositionsJoints = p.calculateInverseKinematics(robot, 7, goal_pos, targetOrientation = Orientation)
p.setJointMotorControlArray(robot, range(6), p.POSITION_CONTROL, targetPositions=targetPositionsJoints)
s = p.getNumJoints(robot)


for i in range(3000):
    p.stepSimulation()
    time.sleep(1. / 60.)
    if i == 120:
        print(p.getLinkState(robot, 7))

