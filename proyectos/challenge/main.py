#Importacion de librerias
import os
import sys
import cv2
import time
import pybullet as p
import pybullet_data
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from MoveOmni import MoveOmni

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10.0)
p.loadURDF("plane.urdf")

# Cargar el robot omni a una altura de 0.12 m
omni_base_pos = [0, 0, 0.12]
manipulador_base_pos = [0, 0, 0.19]

manipulator_id = p.loadURDF("D:/Users/leslie/GitHub/ME5150_tutorials/modelos/manipuladores/brazo_omni_v2_description/urdf/brazo_omni_v2.xacro", 
                            basePosition = manipulador_base_pos, 
                            useFixedBase = True)

num_joints = p.getNumJoints(manipulator_id)
sliders = [p.addUserDebugParameter(f"Link {i+1}", -np.pi, np.pi, 0) for i in range(num_joints)]
#0.8, 0.3, 0.4
omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", basePosition = omni_base_pos, useFixedBase = True, globalScaling = 1.5)
move_omni = MoveOmni([0, 0, 0], vel = 20)
slider_omni = [p.addUserDebugParameter("x_pos", -1, 1, 0), p.addUserDebugParameter("y_pos", -1, 1, 0), p.addUserDebugParameter("w", -2, 2, 0)]

bag_id = p.loadURDF("../modelos/objetos/mini_bag_description/urdf/mini_bag.xacro", basePosition = [0.75, 0, 0.5])
mesa_id = p.loadURDF("../modelos/entornos/table/table.urdf", basePosition = [1, 0, 0.35], globalScaling = 0.5, useFixedBase = True)

# a = p.setCollisionFilterPair(manipulator_id, bag_id, 4, 0, True)
# print(a)
# Variables con valores iniciales, pueden usar estas y/o crear las suyas propias
state = 0         # Controla que aruco se debe buscar
substate = 0      # Controla que accion se debe hacer
t_cam = 0         # Controla la frecuencia de la cámara
t_mov = 0         # Controla la frecuencia de movimiento
img = None        # Imagen de la cámara
next_pose = None  # Siguiente pose del robot

def update_mov():
    """
    Función que actualiza el movimiento del robot omni cada 0.01 segundos
    """
    global t_mov
    global next_pose
    global move_omni

     # Si no está en la posición objetivo
    if t_mov + 0.01 < time.time() and move_omni.is_on_target() == False:
        move_omni.update_pose()

        new_position = np.concatenate([move_omni.act_pose[:2], [0.12]])

        new_orientation = np.concatenate([ [0.0, 0.0], [move_omni.act_pose[2]]])
        new_orientation = p.getQuaternionFromEuler(new_orientation)
        
        p.addUserDebugPoints([new_position], [[0, 1, 0]], pointSize=2, lifeTime=0.001)
        p.resetBasePositionAndOrientation(omni_id, new_position, new_orientation)
        
        new_base_manipulator = np.concatenate([move_omni.act_pose[:2], [0.19]])
        
        new_base_orientation = new_orientation
        p.resetBasePositionAndOrientation(manipulator_id, 
                                new_base_manipulator,
                                 new_base_orientation)

        t_mov = time.time()

    # Si está en la posición objetivo
    if move_omni.is_on_target() == True:
        next_pose = None # Reiniciar la siguiente posición

while True:
    
    omni_pos = p.getBasePositionAndOrientation(omni_id, 0)[0]
    manipulador_pos = p.getBasePositionAndOrientation(manipulator_id)[0]
    # print(manipulador_pos)
    # print(omni_pos)
    # print(omni_pos, manipulador_pos, np.array(manipulador_pos) - np.array(omni_pos))
    q = [p.readUserDebugParameter(slider) for slider in sliders]
    p.setJointMotorControlArray(manipulator_id, range(num_joints), p.POSITION_CONTROL, targetPositions=q)

    position_omni = move_omni.act_pose[:2]
    theta_omni = move_omni.act_pose[2]

    if next_pose is None:
        next_pose = [p.readUserDebugParameter(slider) for slider in slider_omni]
        if next_pose is not None or next_pose != 0.0:
            move_omni.set_target_pose(next_pose)

    update_mov()

    p.stepSimulation()
    # p.performCollisionDetection()
    time.sleep(1./240.)

