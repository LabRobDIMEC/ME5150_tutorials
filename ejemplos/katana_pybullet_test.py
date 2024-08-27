import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0],useFixedBase=useFixedBase)

print(p.getNumJoints(robotId))

p.setGravity(0, 0, -9.81)
p.setTimeStep(1 / 240)

# Create sliders for each joint
sliders = []
sliders.append(p.addUserDebugParameter("Link 1", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 2", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 3", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 4", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 5", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 6", -np.pi, np.pi, 0))
sliders.append(p.addUserDebugParameter("Link 7", -np.pi, np.pi, 0))

# Main simulation loop
while True:
    q = [p.readUserDebugParameter(slider) for slider in sliders]
    # Set the joint angles of the robot
    p.setJointMotorControlArray(robotId, range(4), p.POSITION_CONTROL, targetPositions = q)
    # Step the simulation
    p.stepSimulation()
    time.sleep(1 / 240)

#p.stepSimulation()

