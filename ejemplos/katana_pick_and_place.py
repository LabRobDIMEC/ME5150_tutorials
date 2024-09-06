import pybullet as p
import pybullet_data
import time
import numpy as np

# Conectar al cliente de simulación
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
useFixedBase = True  
p.setGravity(0, 0, -9.81)

# Cargar los URDFs
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("modelos/manipuladores/katana/katana.urdf", basePosition=[0, 0, 0], useFixedBase=useFixedBase)
football_pitch = p.loadURDF("modelos/manipuladores/katana/football_pitch.urdf", basePosition=[1, 1.08, 0.1], useFixedBase=True)
soccerball = p.loadURDF("modelos/manipuladores/katana/soccerball.urdf", basePosition=[0.19, -0.31, 0.2], useFixedBase=False)

# Configuración de obstáculos
collision_objects = []
collision_positions = [[0.1, -0.3, 0.15]]
angle = -0.785398
collision_orientations = [p.getQuaternionFromEuler([angle, 0, 0])] * len(collision_positions)

for pos, orient in zip(collision_positions, collision_orientations):
    collision_id = p.loadURDF("modelos/manipuladores/katana/basket.urdf", pos, orient)
    collision_objects.append({
        'id': collision_id,
        'position': pos,
        'radius': 0.1  # Establecer un radio estimado si es necesario, ajusta según tu modelo
    })
    # Crear restricciones
    p.createConstraint(collision_id, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], parentFramePosition=[0, 0, 0], childFramePosition=pos, childFrameOrientation=orient)

# Obtener el número de articulaciones y filtrar las móviles
num_joints = p.getNumJoints(robotId)
movable_joints = []
gripper_joints = []

for i in range(num_joints):
    joint_info = p.getJointInfo(robotId, i)
    joint_type = joint_info[2]
    joint_name = joint_info[1].decode('utf-8')
    
    if joint_type != p.JOINT_FIXED:  # Excluir articulaciones fijas
        if "finger" not in joint_name:
            movable_joints.append(i)
            print(f"Joint {i}: {joint_name}")
    if "finger" in joint_name:
        gripper_joints.append(i)
        print(f"Joint {i}: {joint_name}, lower limit", p.getJointInfo(robotId, i)[8])
        print(f"Joint {i}: {joint_name}, upper limit", p.getJointInfo(robotId, i)[9])

print("joint ficticio", p.getJointInfo(robotId, 4)[1])

# Parámetros del gripper
gripper_r_joint_index = 6
gripper_l_joint_index = 7
target_position_closed = 0.03  # Quizás cambiar por el radio de la pelota
target_position_open = 0.2
max_force = 50

# Posición y orientación de la pelota
posicion_pelota, orientacion_pelota = p.getBasePositionAndOrientation(soccerball)
print("posicion pelota", posicion_pelota)

def detect_collisions(gripper_id, collision_objects):
    """Detecta colisiones entre el gripper y los objetos de colisión."""
    collision_info = p.getContactPoints(bodyA=gripper_id)
    for obj in collision_objects:
        collision_info += p.getContactPoints(bodyA=gripper_id, bodyB=obj['id'])
    return len(collision_info) > 0

def adjust_target_position(target_position, collision_objects, safety_distance=0.1):
    """Ajusta la posición objetivo para evitar colisiones."""
    adjusted_position = np.array(target_position)
    
    for obj in collision_objects:
        obj_position = np.array(obj['position'])
        obj_radius = obj['radius']
        distance_to_obj = np.linalg.norm(adjusted_position - obj_position)
        
        if distance_to_obj < (obj_radius + safety_distance):
            # Calcular la dirección de ajuste
            direction = adjusted_position - obj_position
            direction = direction / np.linalg.norm(direction)  # Normalizar la dirección
            
            # Ajustar la posición para evitar la colisión
            adjusted_position = obj_position + direction * (obj_radius + safety_distance)
    
    return adjusted_position.tolist()

def move_gripper(robot_id, target_position):
    """Mueve el gripper evitando obstáculos."""
    gripper_wrist = 4
    
    # Ajustar la posición objetivo para evitar colisiones
    if detect_collisions(robot_id, collision_objects):
        #print("Colisión detectada. Ajustando trayectoria.")
        target_position = adjust_target_position(target_position, collision_objects)
    
    # Calcular la cinemática inversa para obtener las posiciones de las juntas
    joint_poses = p.calculateInverseKinematics(robot_id, gripper_wrist, target_position)
    
    # Configurar el control del motor para cada articulación
    for i, joint_index in enumerate(movable_joints):
        p.setJointMotorControl2(robot_id, joint_index, p.POSITION_CONTROL, targetPosition=joint_poses[i], force=max_force)

def set_initial_joint_positions(robot_id):
    initial_joint_positions = [0, 1, 0.66, -1.3, 0]  # Ajusta estos valores según sea necesario
    for i, joint_index in enumerate(movable_joints[:len(initial_joint_positions)]):
        p.resetJointState(robot_id, joint_index, initial_joint_positions[i])
    time.sleep(1)

set_initial_joint_positions(robotId)

# Obtener los índices de los links de los dedos del gripper
gripper_joints = [6, 7]  # Suponiendo que estos son los índices de los dedos del gripper

while True:
    p.stepSimulation()
    # Obtener la posición del end-effector
    end_effector_state = p.getLinkState(robotId, 5)
    end_effector_position = end_effector_state[4]
    
    # Calcular la distancia a la pelota
    distancia_a_pelota = np.linalg.norm(np.array(posicion_pelota) - np.array(end_effector_position))
    
    # Mover el gripper
    move_gripper(robotId, posicion_pelota)
    
    if distancia_a_pelota < 0.005:
        p.setJointMotorControl2(robotId, gripper_r_joint_index, p.POSITION_CONTROL, targetPosition=target_position_closed, force=max_force, maxVelocity=3.0)
        p.setJointMotorControl2(robotId, gripper_l_joint_index, p.POSITION_CONTROL, targetPosition=target_position_closed, force=max_force, maxVelocity=3.0)
        time.sleep(1)
    
    time.sleep(1. / 480.)  # El tiempo entre pasos de simulación
