import pybullet as p
import math
import time
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()
planeID = p.loadURDF("plane.urdf")
conveyor_collision_shape_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.6, 24, 0.2))
conveyor_visual_shape_id = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.6, 24, 0.2), rgbaColor=[0.4, 0.4, 0.4, 1.0])
conveyor = p.createMultiBody(0, conveyor_collision_shape_id, conveyor_visual_shape_id, (0, 0, 0))

platform_collision_id_1 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 0.75, 0.2))
platform_visual_id_1 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 0.75, 0.2), rgbaColor=[1., 1., 1., 1.0])
platform1 = p.createMultiBody(0, platform_collision_id_1, platform_visual_id_1, (-1., 0, 0))

platform_collision_id_2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=(0.15, 0.75, 0.2))
platform_visual_id_2 = p.createVisualShape(p.GEOM_BOX, halfExtents=(0.15, 0.75, 0.2), rgbaColor=[1., 1., 1., 1.0])
platform2 = p.createMultiBody(0, platform_collision_id_2, platform_visual_id_2, (1., 0, 0))

ur1 = p.loadURDF("../maniware/assets/ur/ur5e.urdf", [1., 0.6, 0.2], useFixedBase = 1)
ur2 = p.loadURDF("../maniware/assets/ur/ur5e.urdf", [1., -0.6, 0.2], useFixedBase = 1)
ur3 = p.loadURDF("../maniware/assets/ur/ur5e.urdf", [-1., 0.6, 0.2], useFixedBase = 1)
ur4 = p.loadURDF("../maniware/assets/ur/ur5e.urdf", [-1., -0.6, 0.2], useFixedBase = 1)


blue_cube = p.loadURDF("../maniware/assets/conveyor/blue_cube.urdf", [0, -1., 0.2], useFixedBase = 0)
#s = p.getNumJoints(robot)
#print(s, p.getJointInfo(robot, 3))
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)

p.resetBaseVelocity(blue_cube, linearVelocity=[0., 4., 0.], angularVelocity=[0., 0., 0.])
for _ in range(300):
    p.stepSimulation()
    time.sleep(1. / 10.)








