
import pybullet as p
import numpy as np
import cv2
from info_arucos import *

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

def load_maze():
    pos_laberinto = [0.5, 0.75, 0]
    or_laberinto = p.getQuaternionFromEuler([0, 0, np.pi/2])
    # En caso de no querer cargar el laberinto, comentar la siguiente línea (lo demás funcionaría igual)
    p.loadURDF('../modelos/entornos/laberinto/laberinto.urdf', basePosition = pos_laberinto, baseOrientation = or_laberinto, useFixedBase = True)
    laberinto_origin = np.array([0.5, 0, 0])

    for aruco in info_arucos["arucos_l1"]:
        aruco_info = info_arucos["arucos_l1"][aruco]
        pos = np.array([float(aruco_info["x"]), float(aruco_info["y"]), 0.08]) + laberinto_origin
        orien = p.getQuaternionFromEuler([0, np.pi/2, np.deg2rad(float(aruco_info["rz"]))])
        id = aruco_info["id"]
        p.loadURDF(f"../modelos/objetos/Arucos/A{id}/A{id}.urdf", basePosition = pos, baseOrientation = orien, useFixedBase = True)
