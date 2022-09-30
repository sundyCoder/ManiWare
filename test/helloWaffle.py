import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
robot1 = p.loadURDF("../maniware/assets/turtlebot3/waffle.urdf", [0, 0, 0], useFixedBase = 0)
robot2 = p.loadURDF("../maniware/assets/turtlebot3/waffle.urdf", [3., 0, 0], useFixedBase = 1)
#s = p.getNumJoints(robot)
#print(s, p.getJointInfo(robot, 3))
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

#moving robot
p.resetBaseVelocity(robot1, linearVelocity=[10., 0., 0.], angularVelocity=[0., 0., 0.])
print(p.getNumConstraints())


for _ in range(300):
    p.stepSimulation()
    time.sleep(1. / 10.)
