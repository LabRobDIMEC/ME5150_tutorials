#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
from utils import cvK2BulletP, get_img_cam, load_maze
from info_arucos import *

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from aruco_huntingv2 import ArucoHunting
from MoveOmni import MoveOmni

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setTimeStep(1 / 240)

planeId = p.loadURDF("plane.urdf") # Load the plane into the simulation

load_maze()

omni_base_pos = [0, 0, 0.08]
omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", basePosition = omni_base_pos, useFixedBase = True)

move_omni = MoveOmni([0, 0, 0], 10)

camera_matrix = np.array([[691.,0. , 289.],[0., 690., 264.], [0., 0., 1.]])

hunter = ArucoHunting()
hunter.set_marker_length(0.1) # Dado por cubo simulado
hunter.set_camera_parameters(camera_matrix) # Matriz de la cámara

def update_camera():
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
        print(np.concatenate([position_omni[:2], [0]]), camposition)
        p.addUserDebugPoints([camposition], [[1, 0, 0]], pointSize=2, lifeTime=0.001)

        img, _, _ = get_img_cam(width=480,
                                height=480, 
                                camposition = camposition, 
                                camorientation = camorientation, 
                                cam_mat=camera_matrix)
        t_cam = time.time()
    return img

def update_mov():
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

state = 0         # Variable para controlar que aruco debe buscar
substate = 0      # Variable para controlar que accion debe hacer
t_cam = 0         # Variable para controlar la frecuencia de la cámara
t_mov = 0         # Variable para controlar la frecuencia de movimiento
img = None        # Imagen de la cámara
next_pose = None  # Siguiente pose del robot
target_pose = []

def perm_axes(pos):
    new_pos = [pos[2], -pos[0], pos[1]]
    return new_pos

def get_close_to_aruco(data_aruco, substate):
    aruco_position = data_aruco['position']

    perm_aruco_position = perm_axes(aruco_position)
    dx, dy = perm_aruco_position[:2]
    print('pos_aruco:', dx, dy)
    if substate in [0, 1]:
        p = 0.5
    else:
        p = 1
    next_pose =[(dx-0.2)*p, dy/2, 0]
    return next_pose

def get_rotational_action(id):
    if id in [1, 301, 305, 401, 200]:
        next_pose = [0, 0, np.pi/2]
    elif id in [402]:
        next_pose = [0, 0, -np.pi]
    elif id in [100, 307]:
        next_pose = [0, 0, -np.pi/2]
    elif id in [406]:
        next_pose = [0, 0, 2*np.pi]
    return next_pose

def search_aruco(id, aruco_data):
    for aruco in aruco_data:
        if aruco['id'] == id:
            return aruco
    return None

def get_next_pose(img):
    """Calcular la posición siguiente del robot, basado en la imagen de la cámara
    Args:
        img (np.array): imagen de la cámara RGB
    Returns:
        next_pose (np.array): [x, y, theta] siguiente posición del robot, en metros y radianes, 
                    respecto al sistema de coordenadas global.
                    Si se quiere pasar de posición relativa a absoluta, se debe sumar position_omni.
    """
    global state, substate
    global position_omni   # Posición del robot en metros [x, y]
    global theta_omni # Orientación del robot en radianes [theta]

    action = [0, 0, 0]
    pose_omni = np.concatenate([position_omni, [theta_omni]])
    hunter.update_image(img)
    hunter.update_pose_and_ids()
    arucos_data = hunter.arucos_data

    if state == 0:
        id_a = 1
    elif state == 1:
        id_a = 301
    elif state == 2:
        id_a = 401
    elif state == 3:
        id_a = 402
    elif state == 4:
        id_a = 100
    elif state == 5:
        id_a = 100
    elif state == 6:
        id_a = 305
    elif state == 7:
        id_a = 100
    elif state == 8:
        id_a = 307
    elif state == 9:
        id_a = 200
    elif state == 10:
        id_a = 100
    elif state == 11:
        id_a = 406
    else:
        return pose_omni

    if arucos_data != [] and substate <= 2:
        if search_aruco(id_a, arucos_data) is not None:
            action = get_close_to_aruco(search_aruco(id_a, arucos_data), substate)
            substate += 1
    else:
        action = get_rotational_action(id_a)
        substate = 0
        state += 1

    x0, y0, theta0 = pose_omni
    x1, y1, theta1 = action
    next_pose = [x0 + x1*np.cos(theta0) - y1*np.sin(theta0),
                 y0 + x1*np.sin(theta0) + y1*np.cos(theta0),
                 theta0 + theta1]
    return next_pose

while True:
    update_camera()
    position_omni = move_omni.act_pose[:2]   #p.getBasePositionAndOrientation(omni_id)[0][:2]
    theta_omni = move_omni.act_pose[-1] #p.getEulerFromQuaternion(p.getBasePositionAndOrientation(omni_id)[1])[2]

    if next_pose is None:
        next_pose = get_next_pose(img)
        if next_pose is not None:
            move_omni.set_target_pose(next_pose)

    update_mov()
    p.stepSimulation()
