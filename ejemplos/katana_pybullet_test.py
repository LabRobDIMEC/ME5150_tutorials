import pybullet as p
import pybullet_data
import time
import numpy as np


physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True  
p.setGravity(0, 0, -9.81)


planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0], useFixedBase=useFixedBase)
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[-0.6, 0.68, 0.1],useFixedBase=True)
duck1 = p.loadURDF("modelos/manipuladores/katana/duck_vhacd.urdf", basePosition=[0.6, -0.1, 0.15],useFixedBase=True)
duck2 = p.loadURDF("modelos/manipuladores/katana/duck_vhacd.urdf", basePosition=[0.3, 0.1, 0.1],useFixedBase=True)
duck3 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.5, 0.15, 0.1],useFixedBase=True)
duck3 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.4, -0.1, 0.3],useFixedBase=True)
soccerball1 = p.loadURDF("modelos/manipuladores/katana/soccerball.urdf", basePosition=[0.19, -0.31, 0.2],useFixedBase=False)
soccerball2 = p.loadURDF("modelos/manipuladores/katana/soccerball.urdf", basePosition=[0.2, -0.32, 0.2],useFixedBase=False)
soccerball = p.loadURDF("modelos/manipuladores/katana/soccerball.urdf", basePosition=[0.19, -0.31, 0.2],useFixedBase=False)
basket = p.loadURDF("modelos/manipuladores/katana/basket.urdf", basePosition=[0.1, -0.3, 0.15],useFixedBase=True)

angle= -0.785398
orientation= p.getQuaternionFromEuler([angle, 0, 0])
p.resetBasePositionAndOrientation(basket, [0.1, -0.3, 0.15], orientation)
# Obtener el número de articulaciones y filtrar las móviles
num_joints = p.getNumJoints(robotId)
movable_joints = []
gripper_joints =[]

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    joint_name = joint_info[1].decode('utf-8')
    
    if joint_type != p.JOINT_FIXED:  # Excluir articulaciones fijas
        if "finger" not in joint_name:
            movable_joints.append(i)
            print(f"Joint {i}: {joint_name}")
    if "finger" in joint_name:
        gripper_joints.append(i)


sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(len(movable_joints))]
gripper_slider = p.addUserDebugParameter("Gripper", -1, 1, 0)

# Bucle principal de la simulación
while True:
    q = [p.readUserDebugParameter(slider) for slider in sliders]
    gripper_value=p.readUserDebugParameter(gripper_slider)
    p.setJointMotorControlArray(robotId, movable_joints, p.POSITION_CONTROL, targetPositions=q)
    if gripper_joints:
        gripper_positions = [gripper_value] * len(gripper_joints)
        p.setJointMotorControlArray(robotId, gripper_joints, p.POSITION_CONTROL, targetPositions=gripper_positions)   
    p.stepSimulation()
    time.sleep(1 / 240)

 
