import pybullet as p
import pybullet_data
import time
import numpy as np
from picknplace_lib import *


# Conectar al cliente de simulación
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True  
p.setGravity(0, 0, -9.81)

# Cargar los URDFs
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0], useFixedBase=useFixedBase)
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[1, 1.08, 0], useFixedBase=True)
#soccerball = p.loadURDF("modelos/manipuladores/katana/soccerball.urdf", basePosition=[0.19, -0.31, 0.2], useFixedBase=False)
duck2 = p.loadURDF("modelos/manipuladores/katana/duck_vhacd.urdf", basePosition=[0.3, -0.2, 0],useFixedBase=True)
duck3 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.5, 0.15, 0],useFixedBase=True)
duck4 = p.loadURDF("modelos/manipuladores/katana/duck_orange.urdf", basePosition=[0.35, 0, 0],useFixedBase=True)
basket = p.loadURDF("modelos/manipuladores/katana/basket.urdf", basePosition=[0.1, -0.3, 0.15],useFixedBase=True)

angle= -0.785398
orientation= p.getQuaternionFromEuler([angle, 0, 0])
p.resetBasePositionAndOrientation(basket, [0.1, -0.3, 0.15], orientation)


num_joints = p.getNumJoints(robotId)
movable_joints = []
gripper_joints = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    if joint_info[2] == p.JOINT_REVOLUTE or joint_info[2] == p.JOINT_PRISMATIC:
        movable_joints.append(i)

# Parámetros del gripper
gripper_r_joint_index = 6
gripper_l_joint_index = 7
target_position_closed = 0.03  # Quizás cambiar por el radio de la pelota
target_position_open = 0.2
max_force = 50

lista_de_obstaculos = [duck2, duck3, duck4] #llenar con el resto de obstaculos  
basket_obstacle = [basket]  
grid_size = 0.06 #en terminos de la grilla
world_size = 0.35 #en terminos de la grilla
obstacles_height = 0 #en terminos del mundo
tool_frame_link = 8


# Obtener los índices de los links de los dedos del gripper
gripper_joints = [6, 7]  # Suponiendo que estos son los índices de los dedos del gripper
grid, obstaculos= grid_and_obstacles(grid_size, world_size, obstacles_height, lista_de_obstaculos)


def tool_in_pos_with_nullspace(robotId, endEffectorId, targetPos, threshold, maxIter, lowerLimits, upperLimits, jointRanges, restPoses):
    closeEnough = False
    iter = 0
    dist2 = 1e30
    
    while (not closeEnough and iter < maxIter):
        # Calcular la cinemática inversa con espacio nulo
        jointPoses = p.calculateInverseKinematics(robotId, endEffectorId, targetPos,
                                                  lowerLimits=lowerLimits, 
                                                  upperLimits=upperLimits, 
                                                  jointRanges=jointRanges, 
                                                  restPoses=restPoses)
        p.setJointMotorControlArray(robotId, movable_joints, p.POSITION_CONTROL, targetPositions=jointPoses, forces=[max_force]*len(movable_joints))
        ls = p.getLinkState(robotId, endEffectorId)
        newPos = ls[4]  # Posición global del efector final
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = (dist2 < threshold)
        iter += 1
    return closeEnough


start_mundo = [0.4, -0.6, obstacles_height]
goal_mundo = [0.35, 0.33, obstacles_height]
grip_coordinates= [0.19, -0.31, 0.2]
start=world_to_grid(start_mundo, grid_size, obstacles_height)
print("start", start)
goal=world_to_grid(goal_mundo, grid_size, obstacles_height)
print("goal", goal)
# start = (0.48, -0.24, obstacles_height)
# goal = (0.18, 0.18, obstacles_height)

# for (x,y) in grid:
#     print("x", x, "y", y)
#     p.addUserDebugLine([x, y, obstacles_height+0.05], [x+grid_size, y, obstacles_height+0.05], [1, 0, 0], 5)
#     p.addUserDebugLine([x, y, obstacles_height+0.05], [x, y+grid_size, obstacles_height+0.05], [1, 0, 0], 5)

# # Draw the origin or the first cell in the grid
# p.addUserDebugLine([0, 0, obstacles_height], [grid_size, 0, obstacles_height], [1, 1, 1], 4)
# p.addUserDebugLine([0, 0, obstacles_height], [0, grid_size, obstacles_height], [1, 1, 1], 4)


path = a_star_with_obstacles(start, goal, grid, obstaculos, grid_size, obstacles_height)
print("path", path)
print("largor del path", len(path))

lower_l=[-3.025528, -0.135228, -2.221804, -2.033309, -2.993240]
upper_l=[2.891097, 2.168572, 2.054223, 1.876133, 2.870985]
joint_ranges=[5.916625, 2.303800, 4.276027, 3.909442, 5.864225]
pos_descanso=[0.5, 0.9, -0.5, -1.7]


time.sleep(5)
path_index = 0
#while path_index < len(path):
while True:
    umbral=0.1
    iteraciones_max=100
    # Obtener la siguiente posición del camino
    if path_index < len(path):
        next_position = path[path_index]

        if tool_in_pos_with_nullspace(robotId, tool_frame_link, next_position, umbral, iteraciones_max, lower_l, upper_l, joint_ranges, pos_descanso):
            path_index += 1
    
    p.stepSimulation()
    time.sleep(1 / 240)