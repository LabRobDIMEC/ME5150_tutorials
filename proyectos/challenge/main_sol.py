#Importacion de librerias
import os
import sys
import time
import pybullet as p
import pybullet_data
import numpy as np

sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../herramientas')))
from moveomni import MoveOmni

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
manipulator_id = p.loadURDF("../modelos/manipuladores/brazo_omni_v3_description/urdf/brazo_omni_v3.xacro", 
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

# List of target poses [x0, y0, theta0, x1, y1, z1]



def interpolate_poses(start_pose, end_pose, num_points=25):
    """
    Creates a list of interpolated poses between the start_pose and end_pose.
    
    Parameters:
        start_pose (list): The starting pose [x, y, theta].
        end_pose (list): The ending pose [x, y, theta].
        num_points (int): The number of intermediate points.
    
    Returns:
        list: List of interpolated poses.
    """
    start_pose = np.array(start_pose)
    end_pose = np.array(end_pose)
    
    # Linearly interpolate between start_pose and end_pose
    interpolated_poses = np.linspace(start_pose, end_pose, num_points)
    return interpolated_poses.tolist()

p1 = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0 ,0.0]
p2 = [0.0, 0.0, 0.0, 0.1, 0.72, 0.63, 0.3]
p3 = [0.0, 0.1, 0.0, 0.1, 0.72, 0.63, 0.3]
p4 = [0.45, 0.1, 0.0, 0.1, 0.72, 0.63, 0.3]
p5 = [0.45, 0.1, 0.0, 0.1, 0.72, 0.63, -0.4]
p6 = [0.45, 0.1, np.pi/2, 0.1, 0.72, 0.63, -0.4]
p7 = [0.45, 0.1, np.pi/2, 0.1, 0.72, 0.63, 0.1]
p8 = [0.45, 0.5, np.pi/2, 0.1, 0.72, 0.63, 0.1]
p9 = [0.8, 0.5, np.pi/2, 0.1, 0.72, 0.63, 0.1]
p10 = [0.8, 0.5, np.pi/2, 0.1, 0.65, 0.45, 0]
p11 = [0.8, 0.8, np.pi/2, 0.1, 0.65, 0.45, 0]
p12 = [0.8, 0.8, np.pi/2, 0.1, 0.65, 0.45, 2]
p13 = [0.8, 0.5, np.pi/2, 0.1, 0.65, 0.45, 2]

a1 = interpolate_poses(p1,p2)
a2 = interpolate_poses(p2,p3)
a3 = interpolate_poses(p3,p4)
a4 = interpolate_poses(p4,p5)
a5 = interpolate_poses(p5,p6)
a6 = interpolate_poses(p6,p7)
a7 = interpolate_poses(p7,p8)
a8 = interpolate_poses(p8,p9)
a9 = interpolate_poses(p9,p10)
a10 = interpolate_poses(p10,p11)
a11 = interpolate_poses(p11,p12)
a12 = interpolate_poses(p12,p13)
list_of_next_poses = a1 + a2 + a3 + a4 + a5 + a6 + a7 + a8 + a9 + a10 + a11 + a12

#print(list_of_next_poses)

def update_mov():
    """
    Updates the Omni's movement and moves the manipulator if the target is reached.
    """
    global t_mov, next_pose_mani, next_pose

    if t_mov + 0.01 < time.time() and next_pose is not None:
        joint_states = np.array([state[0] for state in p.getJointStates(manipulator_id,range(5))])
        print(np.abs(np.linalg.norm(joint_states - next_pose_mani)))
        mani_is_on_target = np.abs(np.linalg.norm(joint_states - next_pose_mani)) < 0.1
        if not move_omni.is_on_target():
            move_omni.update_pose()
            new_position = np.concatenate([move_omni.act_pose[:2], [0.12]])
            new_orientation = p.getQuaternionFromEuler([0.0, 0.0, move_omni.act_pose[2]])
            p.resetBasePositionAndOrientation(omni_id, new_position, new_orientation)

            # Update manipulator base position
            manipulator_base = np.concatenate([move_omni.act_pose[:2], [0.19]])
            p.resetBasePositionAndOrientation(manipulator_id, manipulator_base, new_orientation)
            t_mov = time.time()
            
        # If omni target reached, move manipulator
        elif move_omni.is_on_target() and not mani_is_on_target:
            print('move_mani')
            # Inverse kinematics to get joint positions
            p.setJointMotorControlArray(manipulator_id, range(len(next_pose_mani)), p.POSITION_CONTROL, targetPositions=next_pose_mani)
            t_mov = time.time()

        else: # Both are on target
            next_pose = None    
        


# Main loop to process poses
while True:
    if next_pose is None and len(list_of_next_poses)>0:
        next_pose = list_of_next_poses.pop(0)
        next_pose_omni = next_pose[:3]
        next_pose_mani = np.array(next_pose[3:]+ [0.0])
        if next_pose_omni is not None:
            move_omni.set_target_pose(next_pose_omni)
    update_mov()
    p.stepSimulation()

