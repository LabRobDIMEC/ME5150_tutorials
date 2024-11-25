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
next_pose = None  # Siguiente pose del robot

def interpolate_poses(start_pose, end_pose, num_points=100):
    """
    Creates a list of interpolated poses between the start_pose and end_pose.
    
    Parameters:
        start_pose (list): The starting pose [x0, y0, theta0, q1, q2, q3, q4].
        end_pose (list): The ending pose [x0, y0, theta0, q1, q2, q3, q4].
        num_points (int): The number of intermediate points.
    
    Returns:
        list: List of interpolated poses.
    """
    # TODO: Implement the interpolation function

    interpolated_poses = []

    return interpolated_poses

""" List of target poses:
- x0, y0, theta0: Pose of the omni base [m, m, rad]
- q1, q2, q3, q4: Joint positions of the manipulator [rad, rad, rad, rad]
"""

# Example of poses:
#  TODO: Modify the poses to create a list of interpolated poses
# Objective: Grab the bag and move it to the other table, with 
poses = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,0.0],
        [0.0, 0.0, 0.0, 0.1, 0.72, 0.63, 0.4],
        [0.2, 0.3, 0.4, 0.1, 0.72, 0.63, 0.4]
    ]

# TODO: Create a list of interpolate poses from the given poses
list_of_next_poses = poses

def update_mov():
    """
    Updates the Omni's movement and moves the manipulator if the target is reached.
    """
    global t_mov, next_pose_mani, next_pose

    if t_mov + 0.01 < time.time() and next_pose is not None:
        # Check if manipulator is on target, given a threshold of 0.05
        joint_states = np.array([state[0] for state in p.getJointStates(manipulator_id,range(5))])
        mani_is_on_target = np.abs(np.linalg.norm(joint_states - next_pose_mani)) < 0.05

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
            # Directly set the joint positions
            p.setJointMotorControlArray(manipulator_id, range(len(next_pose_mani)), p.POSITION_CONTROL, targetPositions=next_pose_mani)
            t_mov = time.time()

        else: # Both are on target
            next_pose = None    
 
# Main loop to process poses
while True:
    if next_pose is None and len(list_of_next_poses)>0:
        # Take the next pose in the list
        next_pose = list_of_next_poses.pop(0)
        # Separate the omni and manipulator poses
        next_pose_omni = next_pose[:3]
        next_pose_mani = np.array(next_pose[3:]+ [0.0])
        # Set the target pose for the omni
        if next_pose_omni is not None:
            move_omni.set_target_pose(next_pose_omni)
    update_mov()
    p.stepSimulation()

