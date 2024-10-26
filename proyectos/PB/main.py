#Importacion de librerias
import os
import sys
import cv2
import time
import pybullet as p
import pybullet_data
import numpy as np
from info_arucos import *
from utils import cvK2BulletP, get_img_cam, load_maze

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from aruco_huntingv2 import ArucoHunting
from MoveOmni import MoveOmni

## INICIALIZACIÓN DE VARIABLES Y OBJETOS

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setTimeStep(1 / 240)

p.loadURDF("plane.urdf") # Cargar el plano
load_maze() # Cargar el laberinto con arucos

# Cargar el robot omni a una altura de 0.08 m
omni_base_pos = [0, 0, 0.08]
omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", basePosition = omni_base_pos, useFixedBase = True)

# Clase que controla las posiciones del robot
move_omni = MoveOmni([0, 0, 0], vel = 30)

# Matriz de la cámara
camera_matrix = np.array([[691.,0. , 289.],[0., 690., 264.], [0., 0., 1.]])

# Clase que detecta arucos
hunter = ArucoHunting()
hunter.set_marker_length(0.1) # Dado por cubo simulado
hunter.set_camera_parameters(camera_matrix)


## LOOP PRINCIPAL

# Variables con valores iniciales, pueden usar estas y/o crear las suyas propias
state = 0         # Controla que aruco se debe buscar
substate = 0      # Controla que accion se debe hacer
t_cam = 0         # Controla la frecuencia de la cámara
t_mov = 0         # Controla la frecuencia de movimiento
img = None        # Imagen de la cámara
next_pose = None  # Siguiente pose del robot

# Funciones para actualizar la cámara y el movimiento del robot
def update_camera():
    """
    Función que actualiza la imagen de la cámara cada 0.01 segundos
    """
    global img
    global t_cam
    global move_omni

    # Cada 0.01 segundos se actualiza la imagen de la cámara
    if t_cam + 0.01 < time.time():
        position_omni = move_omni.act_pose
        theta_rad = position_omni[2]
        theta_deg = np.rad2deg(theta_rad)

        camorientation = np.array([theta_deg, 0, 0])
        camposition = np.concatenate([position_omni[:2], [0]]) + [0.05 * np.cos(theta_rad), 0.05 * np.sin(theta_rad), 0.1] # Posición de la cámara respecto al robot
        p.addUserDebugPoints([camposition], [[1, 0, 0]], pointSize=2, lifeTime=0.001)

        img, _, _ = get_img_cam(width=480,
                                height=480, 
                                camposition = camposition, 
                                camorientation = camorientation, 
                                cam_mat=camera_matrix)
        t_cam = time.time()
    return img

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

        new_position = np.concatenate([move_omni.act_pose[:2], [0.08]])
        new_orientation = np.concatenate([ [0.0, 0.0], [move_omni.act_pose[2]]])
        new_orientation = p.getQuaternionFromEuler(new_orientation)

        p.addUserDebugPoints([new_position], [[0, 1, 0]], pointSize=2, lifeTime=0.001)
        p.resetBasePositionAndOrientation(omni_id, new_position, new_orientation)
        t_mov = time.time()

    # Si está en la posición objetivo
    if move_omni.is_on_target() == True:
        next_pose = None # Reiniciar la siguiente posición

# TODO: Función para calcular la siguiente posición del robot, por completar
def get_next_pose(img):
    """Calcular la posición siguiente del robot, basado en la imagen de la cámara
    Args:
        img (np.array): imagen de la cámara RGB
    Returns:
        next_pose (np.array): [x, y, theta] siguiente posición del robot, en metros y radianes, 
                    respecto al sistema de coordenadas global.
    """

    global state, substate
    global position_omni   # Posición del robot en metros [x, y]
    global theta_omni # Orientación del robot en radianes [theta]

    action = [0, 0, 0] # Acción por defecto
    pose_omni = np.concatenate([position_omni, [theta_omni]])

    # A: Detectar arucos en la imagen

    # B: Filtrar el arucos de interes
    # Hint: Se puede usar la variable state para controlar que aruco se debe buscar

    # C: Con la detección del aruco de interes, calcular la acción a realizar
    # Hint: Para una misma detección de aruco, se pueden realizar varias acciones y para ello se puede usar substate

    # D: Calcular la siguiente posición del robot
    x0, y0, theta0 = pose_omni # Posición actual del robot
    x1, y1, theta1 = action # Acción a realizar con respecto a las coordenadas del robot

    # E: Entregar la siguiente posición [x, y, theta] del robot (en coordenadas globales), usando la posición actual y la acción a realizar
    next_pose = pose_omni # Modificar 
    return next_pose


while True:
    update_camera()
    position_omni = move_omni.act_pose[:2]
    theta_omni = move_omni.act_pose[2]

    if next_pose is None:
        next_pose = get_next_pose(img)
        if next_pose is not None:
            move_omni.set_target_pose(next_pose)

    update_mov()
    p.stepSimulation()
