#Importacion de librerias
import os
import sys
import pybullet as p
import pybullet_data
import time
import numpy as np
import cv2
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from aruco_huntingv2 import ArucoHunting
from MoveOmni import MoveOmni

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setTimeStep(1 / 240)

planeId = p.loadURDF("plane.urdf") # Load the plane into the simulation

laberintoId = p.loadURDF('../modelos/entornos/laberinto/laberinto.urdf', basePosition = [4, 0, 0])
base_pos = [0, 0, 0.085]
omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", basePosition = base_pos, useFixedBase = True)

move_omni = MoveOmni([0, 0, 0], 4)

camera_matrix = np.array([[691.,0. , 289.],[0., 690., 264.], [0., 0., 1.]])
# hunter.set_marker_length(0.078) # Dado por cubo simulado
# hunter.set_camera_parameters(camera_matrix) # Matriz de la cámara

# No modificar - Calculo de matriz de proyeccion de camara
def cvK2BulletP(width, height, cam_mat, near, far):
    fx, fy, cx, cy = cam_mat[0, 0], cam_mat[1, 1], cam_mat[0, 2], cam_mat[1, 2]
    aspect = width / height
    fov = 2 * np.arctan(height / (2 * fy)) * 180 / np.pi
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
    return projection_matrix

# No modificar - Obtener imagen de camara simulada
def get_img_cam(width=480,
                height=480, 
                fov=60, 
                near=0.1, 
                far=100, 
                camposition=[0, 0, 1.5], 
                camorientation = [0, 0, 0], 
                distance=0.1, 
                cam_mat = None):

    aspect = width / height
    yaw, pitch, roll = camorientation + [-90, 0, 0]
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camposition, distance, yaw, pitch, roll, upAxisIndex = 2)
    projection_matrix = cvK2BulletP(width, height, cam_mat, near, far)
    _, _, rgbaImg, depthImg, segImg = p.getCameraImage(width, height, view_matrix, projection_matrix, renderer=p.ER_BULLET_HARDWARE_OPENGL,flags = p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX)
    rgbaImg = cv2.cvtColor(rgbaImg, cv2.COLOR_BGR2RGB)
    return rgbaImg, segImg, depthImg

def update_camera():
    global img
    global t_cam
    global move_omni

    # Cada 0.01 segundos se actualiza la imagen de la cámara
    if t_cam + 0.01 < time.time():
        pos_omni = move_omni.act_pose
        theta_rad = pos_omni[2]
        theta_deg = np.rad2deg(theta_rad)

        camorientation = np.array([theta_deg, 0, 0])
        camposition = np.concatenate([pos_omni[:2], [0]]) + [0.05 * np.cos(theta_rad), 0.05 * np.sin(theta_rad), 0.1] # Posición de la cámara respecto al robot
        p.addUserDebugPoints([camposition], [[1, 0, 0]], pointSize=2, lifeTime=0.01)

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

        new_position = np.concatenate([move_omni.act_pose[:2], [0.085]])
        new_orientation = np.concatenate([ [0.0, 0.0], [move_omni.act_pose[2]]])
        new_orientation = p.getQuaternionFromEuler(new_orientation)

        p.addUserDebugPoints([new_position], [[0, 1, 0]], pointSize=2, lifeTime=0.01)
        p.resetBasePositionAndOrientation(omni_id, new_position, new_orientation)
        t_mov = time.time()

    # Si está en la posición objetivo
    if move_omni.is_on_target() == True:
        next_pose = None # Reiniciar la siguiente posición


state = 0        # Variable para controlar
t_cam = 0         # Variable para controlar la frecuencia de la cámara
t_mov = 0         # Variable para controlar la frecuencia de movimiento
img = None        # Imagen de la cámara
next_pose = None  # Siguiente pose del robot

def get_next_pose(img):
    """Calcular la posición siguiente del robot, basado en la imagen de la cámara
    Args:
        img (np.array): imagen de la cámara RGB
    Returns:
        next_pose (np.array): [x, y, theta] siguiente posición del robot, en metros y radianes, 
                    respecto al sistema de coordenadas global.
                    Si se quiere pasar de posición relativa a absoluta, se debe sumar pos_omni.
    """
    global state
    global pos_omni
    global theta_omni
    print("Pose omni: ", pos_omni)
    pose_omni = np.concatenate([pos_omni, [theta_omni]])
    if state == 0:
        next_pose = [0.5, 1, 0]
        state += 1
    elif state == 1:
        next_pose = [0.5, 0, 0]
        state += 1
    elif state == 2:
        next_pose = [1, 0, 1.4]
        state += 1
    elif state >= 3:
        next_pose = [0, 0, -1.4]

    next_pose = np.array(next_pose) + pose_omni
    return next_pose


while True:
    update_camera()
    pos_omni = p.getBasePositionAndOrientation(omni_id)[0][:2]
    theta_omni = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(omni_id)[1])[2]

    if next_pose is None:
        next_pose = get_next_pose(img)
        move_omni.set_target_pose(next_pose)
    
    update_mov()
    p.stepSimulation()