import numpy as np
import pybullet as p
import pybullet_data

#########-------- FUNCIONES UTILES -----#####

# Simulacion en pybullet
physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Set gravity and time step
p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(1)

# Load SCARA robot arm
planeId = p.loadURDF("plane.urdf")
initialori = p.getQuaternionFromEuler([0, 0, np.deg2rad(90)]) # initial orientation of the robot

robotId = p.loadURDF("modelos/manipuladores/scara/scara.urdf",
                     basePosition = [0, 0, 0], 
                     baseOrientation = initialori, 
                     useFixedBase = True)

"""
Calcula la matriz jacobiana de una funcion f en un punto x
Parametros: 
    f: función de la que se calcula la matriz jacobiana (vector)
    x: punto en que se evalua la matriz jacobiana (vector)
    epsilon: error admisible para calcular la derivada (escalar)
Retorna:
    jacobian_matrix: matriz jacobiana de la función en el punto x (matriz)
"""
def jacobian(f, x, epsilon = 1e-1):
    n = len(x)
    jacobian_matrix = np.zeros((n, n))
    f_x = f(x)
    
    for i in range(n):
        perturbation = np.zeros(n)
        perturbation[i] = epsilon
        f_x_plus = f(x + perturbation)
        jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
        
    return jacobian_matrix


#####################------DESARROLLO AUX ------

# Link index of the end effector
ef_idx = 2

# Largos de los eslabones del SCARA, en metros
L1 = 0.330
L2 = 0.336


"""
Se define la CINEMÁTICA DIRECTA del manipulador SCARA para obtener la posición del efector final
"""
def F(theta):
    f1 = L1 * np.cos(theta[0]) + L2 * np.cos(theta[0] + theta[1])
    f2 = L1 * np.sin(theta[0]) + L2 * np.sin(theta[0] + theta[1])
    f3 = theta[2] + 0.26
    return np.array([f1, f2, f3])

"""
NOTAS DEL MÉTODO:
Resolucion cinematica inversa con el metodo de Newton Raphson.
Se busca encontrar los valores de los joints que cumplen con una posicion deseada del efector final.

Función: NewtonRapson(f, q0, pd)
Parametros:
- f: es la funcion de la cinemática directa del manipulador
- q0: [q1, q2, q3] los valores iniciales de los joints 
- pd: [x, y, z] es la posicion deseada del efector final
- tol: float, es la tolerancia para la convergencia 
- max_iter: float, es el numero maximo de iteraciones
Retorna:
- q_new: [q1, q2, q3] los valores de los joints que cumplen con la posicion deseada
"""

def NewtonRaphson(f, q0, pd, tol=1e-3, max_iter=1000):
    q = q0 # Paso 1: Setear el punto de inicio. q0 será el initial guess
    
    for _ in range(max_iter):
        # Paso 2: Calcular jacobiano de f en q
        jacobiano_val = jacobian(f, q)

        # Paso 3: Calcular la matriz inversa del jacobiano
        inv_jacobiano = np.linalg.inv(jacobiano_val)

        # Paso 4: Aplicar método Newton Raphson
        if np.abs(np.linalg.det(jacobiano_val)) > tol:
            q_new = q - np.dot(inv_jacobiano, f(q) - pd)

        # Si el determinante es cercano a 0, se agrega ruido al vector q_new, para que no diverja 
        else:
            q_new = q + np.random.rand(len(q)) #se mueve q aleatoriamente

        # Paso 5: Condición de convergencia
        # Si la norma de la diferencia entre q_new y q es menor que la tolerancia, se alcanzó la convergencia
        if np.linalg.norm(q_new - q) < tol:
            # Se retorna el valor de q_new
            return q_new 
        q = q_new

    # Si no se alcanzó la convergencia después de max_iter iteraciones, se levanta un error
    raise ValueError("No se pudo encontrar la solución después de %d iteraciones" % max_iter)


def adecuacion_q(q):
    """
    Se realiza una transformacion de los angulos para que estén restringidos
    a un rango de -pi a pi
    """
    q_rad = [np.mod(q[0] + np.pi, 2 * np.pi) - np.pi, 
            np.mod(q[1] + np.pi, 2 * np.pi) - np.pi, 
            q[2]]
    return q_rad



# CINEMÁTICA INVERSA PYBULLET
def IK_pybullet(xyz):
    x, y, z = xyz
    q = p.calculateInverseKinematics(
            robotId, endEffectorLinkIndex = ef_idx, targetPosition=[x, y, z])
    return q

# CINEMÁTICA DIRECTA NEWTON RAPHSON
def IK_NR(xyz):

    q0 = np.array([np.deg2rad(15), np.deg2rad(15), 0.10]) # initial guess

    q_deg = NewtonRaphson(F, q0 = q0, pd = np.array(xyz))
    q = adecuacion_q(q_deg)
    return q

while True:
    xyz = np.array([0.50, 0.2, 0.15]) # en m

    "Cambiar el metodo de resolucion de la cinematica inversa, comentando el que no se usara"
    q = IK_NR(xyz)
    # q = IK_pybullet(xyz)
    print(q)
    p.setJointMotorControlArray(bodyUniqueId = robotId,
                        jointIndices = range(p.getNumJoints(robotId)),
                        controlMode = p.POSITION_CONTROL,
                        targetPositions = q)
    p.stepSimulation()