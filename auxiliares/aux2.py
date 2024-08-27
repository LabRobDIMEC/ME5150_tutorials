import pybullet as p
import pybullet_data
import time
import numpy as np

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setRealTimeSimulation(1) # Set the simulation to be real time

planeId = p.loadURDF("plane.urdf") # Load the plane into the simulation

# Load the roboticArmURDF into the simulation
robotic_arm = p.loadURDF("modelos/manipuladores/twolinks/twolinks.urdf", basePosition=[0, 0, 0], useFixedBase=True)

# Define the joint indices for each link of the robot
num_joints = p.getNumJoints(robotic_arm)
print(num_joints)

link_indices = [joint_index for joint_index in range(num_joints)]
print(link_indices)

# Create sliders for each joint
sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(num_joints)]

def move_robot():
    while True:

        ## Inversa
        xyz = [1, 1, 1]
        
        # Calcular angulos de joints con cinematica inversa
        # angles = p.calculateInverseKinematics(robotic_arm, 2, xyz)
        # print(angles)

        ## Directa

        # Obtener los valores de angulos del slider
        angles = [p.readUserDebugParameter(slider) for slider in sliders]
        # print(angles)

        #op1: setear el estado de 1 de los motores a la vez
        # Set the joint angles of the robot
        # for i, angle in enumerate(angles):
        #     p.setJointMotorControl2(robotic_arm, i, p.POSITION_CONTROL, targetPosition=angle)

        #op2: setear el estado de todos los motores a la vez
        # p.setJointMotorControlArray(robotic_arm, link_indices, p.POSITION_CONTROL, targetPositions=angles)
        
        # Step the simulation
        p.stepSimulation()
        time.sleep(1/240)  # Control the simulation speed

# Call the move_robot function
move_robot()


"""
Actividades (una a la vez, en el orden dado):
1. Mueve el robot de dos links con sliders
2. Mueve el robot de dos link a un punto determinado en código, sin sliders

3. Quitar el robot de dos links del simulador

4. Elige otro manipulador y agregalo al simulador
5. Agregar un objeto al simulador, dentro del espacio de trabajo del segundo robot

6. Mueve la punta del manipulador cerca del objeto usando cinematica directa
7. Mueve la punta del manipulador cerca del objeto usando cinematica inversa
"""