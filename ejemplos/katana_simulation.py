import pybullet as p
import pybullet_data
import time
import numpy as np

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True #Para que el brazo no flote o se desplace

planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0],useFixedBase=True)
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[-0.6, 0.68, 0.1],useFixedBase=True)
duck = p.loadURDF("modelos/manipuladores/katana/duck_vhacd.urdf", basePosition=[0.6, -0.1, 0.1],useFixedBase=True)

num_joints = p.getNumJoints(robotId)

#obtener informacion del gripper:
gripper_id = 6
gripper_info = p.getLinkState(robotId, gripper_id)
print("gipper", gripper_info)

desired_orientations = [
    (0.0, 0.0, 0.6525858044624329, 0.7577148675918579),  # Link 0
    (0.7046929001808167, -0.058392561972141266, 0.7055594325065613, 0.046733032912015915),  # Link 1
    (0.9954941868782043, -0.07468743622303009, 0.05828884616494179, -0.003931256942451),  # Link 2
    (0.8961437344551086, -0.06316094845533371, -0.4374159872531891, -0.04005450755357742),  # Link 3
    (-0.3234157860279083, 0.029727399349212646, 0.9451802968978882, 0.03)  # Link 4
]

trajectory_points = [
    [0.1, 0.0, 0.11], [0.2, 0.0, 0.11], [0.3, 0.0, 0.11], [0.4, 0.0, 0.11],
    [0.5, 0.0, 0.11], [0.6, 0.0, 0.11], [0.7, 0.0, 0.11], [0.8, 0.0, 0.11],
    [0.9, 0.0, 0.11], [1.0, 0.0, 0.11], [1.1, 0.0, 0.11], [1.2, 0.0, 0.11],
    [1.3, 0.0, 0.11], [1.4, 0.0, 0.11], [1.5, 0.0, 0.11], [1.6, 0.0, 0.11],
    [1.7, 0.0, 0.11], [1.8, 0.0, 0.11], [1.9, 0.0, 0.11], [2.0, 0.0, 0.11]
]



movable_joints = []
for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    if joint_type != p.JOINT_FIXED:  # Excluir articulaciones fijas
        movable_joints.append(i)


for idx, point in enumerate(trajectory_points):
    if idx == 0:
        # Usar las orientaciones deseadas para el primer punto
        target_positions = p.calculateInverseKinematics(robotId, movable_joints[-1], point, desired_orientations[-1])
        for i, joint in enumerate(movable_joints):
            if i < len(desired_orientations):
                p.resetJointState(robotId, joint, target_positions[i], targetVelocity=0)
                p.resetBasePositionAndOrientation(robotId, point, desired_orientations[i])
    else:
        # Para los puntos restantes, usa la orientación del primer punto
        target_positions = p.calculateInverseKinematics(robotId, movable_joints[-1], point)

    # Configurar los ángulos de las juntas del robot para las articulaciones móviles
    p.setJointMotorControlArray(robotId, movable_joints, p.POSITION_CONTROL, targetPositions=target_positions[:len(movable_joints)])
    
    # Avanzar la simulación durante un tiempo para que el brazo se mueva
    for _ in range(240):  # Ajustar el rango para controlar la duración de la simulación
        p.stepSimulation()
        time.sleep(1./240.)