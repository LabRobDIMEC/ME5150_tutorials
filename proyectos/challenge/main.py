#Importacion de librerias
import os
import sys
import cv2
import time
import pybullet as p
import pybullet_data
import numpy as np

# Connect to the PyBullet simulator
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath()) # Set the search path to find the plane.urdf file

p.setGravity(0,0,-9.81) # Set the gravity to be in the negative z direction
p.setTimeStep(1 / 240)

p.loadURDF("plane.urdf") # Cargar el plano
# Cargar el robot omni a una altura de 0.08 m
omni_base_pos = [0, 0, 0.07]
manipulador_base_pos = [0, 0, 0.19]
manipulador_id = p.loadURDF("../modelos/manipuladores/brazo_omni_description/urdf/brazo_omni.xacro", basePosition = manipulador_base_pos, useFixedBase = True)
omni_id = p.loadURDF("../modelos/mini_omni/urdf/mini_omni.xacro", basePosition = omni_base_pos, useFixedBase = True, globalScaling = 1.5)
bag_id = p.loadURDF("../modelos/objetos/mini_bag_description/urdf/mini_bag.xacro", basePosition = [1, 0, 0.5])
mesa_id = p.loadURDF("../modelos/entornos/table/table.urdf", basePosition = [1, 0, 0.35], globalScaling = 0.5, useFixedBase = True)

while True:

    p.stepSimulation()
    time.sleep(1./240.)

