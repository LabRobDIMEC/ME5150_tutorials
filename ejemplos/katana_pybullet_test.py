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
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[1, 1.08, 0.1],useFixedBase=True)
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
        print(f"Joint {i}: {joint_name}, lower limit", p.getJointInfo(robotId, i)[8])
        print(f"Joint {i}: {joint_name}, upper limit", p.getJointInfo(robotId, i)[9])

#setear parametros para que el gripper funcione correctamente
gripper_r_joint_index= 6
gripper_l_joint_index= 7

target_position_closed= 0.03 #quizas cambiar por el radio de la pelota
target_position_open= 0.2
max_force= 5

p.setJointMotorControl2(robotId, gripper_r_joint_index, p.POSITION_CONTROL, targetPosition= target_position_closed, force= max_force)
p.setJointMotorControl2(robotId, gripper_l_joint_index, p.POSITION_CONTROL, targetPosition= -target_position_closed, force= max_force)



sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(len(movable_joints))]
gripper_slider = p.addUserDebugParameter("Gripper", -1, 1, 0)

# Bucle principal de la simulación
while True:
    # Avanza un paso en la simulación
    p.stepSimulation()
    
    # Lógica para controlar el gripper, por ejemplo con una condición
    # (Esto es solo un ejemplo; puedes ajustar cuándo ejecutas esta acción)
    current_time = time.time()
    
    if current_time % 10 < 5:  # Cierra el gripper cada 5 segundos
        p.setJointMotorControl2(bodyUniqueId=robotId,
                                jointIndex=gripper_r_joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position_closed,
                                force=max_force)
        
        p.setJointMotorControl2(bodyUniqueId=robotId,
                                jointIndex=gripper_l_joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position_closed,
                                force=max_force)
    else:  # Abre el gripper los siguientes 5 segundos
        p.setJointMotorControl2(bodyUniqueId=robotId,
                                jointIndex=gripper_r_joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position_open,
                                force=max_force)
        
        p.setJointMotorControl2(bodyUniqueId=robotId,
                                jointIndex=gripper_l_joint_index,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=target_position_open,
                                force=max_force)

    time.sleep(1. / 240.)  # El tiempo entre pasos de simulación