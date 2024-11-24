#Importacion de librerias
import os
import sys
import time
import pybullet as p
import pybullet_data
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from MoveOmni import MoveOmni

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.loadURDF("plane.urdf")

# Cargar la omni y el manipulador
omni_base_pos = [0, 0, 0.06]
manipulador_base_pos = [0, 0, 0.19]

omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", 
                            basePosition = omni_base_pos, 
                            useFixedBase = True, 
                            globalScaling = 1.5)
manipulator_id = p.loadURDF("../modelos/manipuladores/brazo_omni_v2_description/urdf/brazo_omni_v2.xacro", 
                            basePosition = manipulador_base_pos, 
                            useFixedBase = True)

# Carga de objetos
p.loadURDF("../modelos/objetos/mini_bag_description/urdf/mini_bag.xacro", basePosition = [0.75, 0.1, 0.5])
p.loadURDF("soccerball.urdf", globalScaling = 0.1, basePosition = [0, 0.8, 0.1])
p.loadURDF("objects/mug.urdf", basePosition = [0.6, 1, 0.5])
p.loadURDF("../modelos/entornos/table/table.urdf", basePosition = [1, 0, 0.35], globalScaling = 0.5, useFixedBase = True)
p.loadURDF("../modelos/entornos/table/table.urdf", basePosition = [0.8, 1, 0.35], globalScaling = 0.5, useFixedBase = True)
p.loadURDF("../modelos/objetos/vaso.urdf", basePosition = [0.9, 0.2, 0.5])

num_joints = p.getNumJoints(manipulator_id) - 1
move_omni = MoveOmni([0, 0, 0], vel = 5)

# Variables con valores iniciales
t_mov = 0         # Controla la frecuencia del movimiento
next_pose = None  # Siguiente pose del robot base

list_of_next_poses = [[0.5, 0, 0, None, None, None]] # Completar con las posiciones objetivo [x0, y0, theta0, x1, y1, z1] en metros y radianes

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
    
    # omni_pos = p.getBasePositionAndOrientation(omni_id, 0)[0]
    # manipulador_pos = p.getBasePositionAndOrientation(manipulator_id)[0]

    # q = [p.readUserDebugParameter(slider) for slider in sliders]
    # p.setJointMotorControlArray(manipulator_id, range(num_joints), p.POSITION_CONTROL, targetPositions=q)

    # Esto mueve la omnidireccional
    position_omni = move_omni.act_pose[:2]
    theta_omni = move_omni.act_pose[2]

    if next_pose is None:
        next_pose = list_of_next_poses.pop(0)
        if next_pose is not None or next_pose != 0.0:
            move_omni.set_target_pose(next_pose)

    update_mov()

    p.stepSimulation()
    time.sleep(1./240.)

