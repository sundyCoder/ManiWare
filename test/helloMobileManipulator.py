import pybullet as p
import math
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")

dingo = p.loadURDF("../maniware/assets/dingo/dingo-o.urdf", [1., 1., 0.], useFixedBase = 0)
constrain_ground = p.createConstraint(planeID, -1, dingo, -1, p.JOINT_FIXED, None,
                                                         [1., 1., 0.], [0., 0., 0.])
# waffle = p.loadURDF("../assets/turtlebot3/waffle.urdf", [0, 0, 0], useFixedBase = 0)
# manipulator = p.loadURDF("../assets/open_manipulator/manipulator.urdf", [-0.092, 0, 0.091], useFixedBase = 0)
manipulator = p.loadURDF("../maniware/assets/ur5/ur5.urdf", [0., 0., 1.], useFixedBase = 0)

joint = p.createConstraint(dingo, -1, manipulator, -1, p.JOINT_FIXED, None, [0., 0., 0.1], [0., 0., 0.])

p.setTimeStep(1. / 240)
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(0)

#moving robot

p.resetBaseVelocity(dingo, linearVelocity=[2., 2., 0.], angularVelocity=[0., 0., np.pi/12])
print(p.getNumConstraints())

for i in range(2400):
    if constrain_ground and i > 10:
        p.removeConstraint(constrain_ground)
        constrain_ground = None
    p.stepSimulation()
    time.sleep(0.05)
