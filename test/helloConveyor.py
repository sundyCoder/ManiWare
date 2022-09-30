import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
conveyor = p.loadURDF("../maniware/assets/conveyor/conveyor.urdf", [0, 0, 0], useFixedBase = 1)
blue_cube = p.loadURDF("../maniware/assets/conveyor/blue_cube.urdf", [0, -0.5, 0.74], useFixedBase = 0)
#s = p.getNumJoints(robot)
#print(s, p.getJointInfo(robot, 3))
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

p.resetBaseVelocity(blue_cube, linearVelocity=[0., 4., 0.], angularVelocity=[0., 0., 0.])
for _ in range(300):
    p.stepSimulation()
    time.sleep(1. / 10.)
