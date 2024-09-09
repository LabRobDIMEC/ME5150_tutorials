"""
NOMBRE: __________Completar__________
"""
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider



#####---------FUNCIONES ÚTILES---------####

"""
Esta función de jacobiano está lista, se modificó con respecto a la del auxiliar 4, para que ahora 
pueda recibir matrices rectangulares, como es en este caso.
Usar solo si lo estima conveniente (depende del método de resolución que elija).
"""
def jacobian(f, x, epsilon=1e-1): 
        n = len(x)
        m = len(f(x))
        jacobian_matrix = np.zeros((m, n))
        f_x = f(x)
        
        for i in range(n):
            perturbation = np.zeros(n)
            perturbation[i] = epsilon
            f_x_plus = f(x + perturbation)
            jacobian_matrix[:, i] = (f_x_plus - f_x) / epsilon
            
        return jacobian_matrix


class SCARAPrisPoseViewer():
    def __init__(self, val_list = [0, 0, 0]):     
        self.val_list = val_list
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin(self):
        
        # np.linalg.multi_dot([Z1, X1, ...., Zn, Xn]), para rotaciones: rot_axis(np.deg2rad(self.values_list[i]))

        # Eje del origen
        origen = np.identity(4) 

        # Primer link, del suelo al primer eje
        Z1 = np.linalg.multi_dot([self.translation_z(320), self.rot_z(np.deg2rad(self.val_list[0]))])
        X1 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(90))])
        Z2 = np.linalg.multi_dot([self.translation_z(-120), self.rot_z(np.deg2rad(self.val_list[1]-90))])
        X2 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(-90))])
        Z3 = np.linalg.multi_dot([self.translation_z(self.val_list[2] + 325), self.rot_z(np.deg2rad(0))])
        X3 = np.linalg.multi_dot([self.translation_x(0), self.rot_x(np.deg2rad(-90))])
        A1 = np.linalg.multi_dot([origen,Z1])
        A2 = np.linalg.multi_dot([A1,X1,Z2])
        A3 = np.linalg.multi_dot([A2,X2,Z3,X3])

        # Agregar las poses de todos los ejes a la lista de poses
        poses = np.array([origen, A1, A2, A3])
        self.poses = poses
        return self.poses

    def translation_x(self,x):
        return np.array([[1, 0, 0, x], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_y(self,y):
        return np.array([[1, 0, 0, 0], [0, 1, 0, y], [0, 0, 1, 0], [0, 0, 0, 1]])
    
    def translation_z(self, z):
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, z], [0, 0, 0, 1]])

    def rot_x(self, qx):
        return np.array([[1, 0, 0, 0], [0, np.cos(qx), -np.sin(qx), 0], [0, np.sin(qx), np.cos(qx), 0], [0, 0, 0, 1]])

    def rot_y(self, qy):
        return np.array([[np.cos(qy), 0, np.sin(qy), 0], [0, 1, 0, 0], [-np.sin(qy), 0, np.cos(qy), 0], [0, 0, 0, 1]])

    def rot_z(self, qz):
        return np.array([[np.cos(qz), -np.sin(qz), 0, 0], [np.sin(qz), np.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def draw_axes_tf(self, poses, name="", color="k"):
        for pose in poses:
            self.ax3.set_xlim(-700, 700)
            self.ax3.set_ylim(-700, 700)
            self.ax3.set_zlim(0, 600)
            self.ax3.set_xlabel('x-axis')
            self.ax3.set_ylabel('y-axis')
            self.ax3.set_zlabel('z-axis')
            self.ax3.scatter(xs=[0], ys=[0], zs=[0], marker='o', color=color)
            origin_pose = np.transpose(pose)[3, 0:3]            
            x_rot = np.linalg.multi_dot([pose, [1, 0, 0, 0]])
            y_rot = np.linalg.multi_dot([pose, [0, 1, 0, 0]])
            z_rot = np.linalg.multi_dot([pose, [0, 0, 1, 0]])
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], x_rot[0],
                    x_rot[1], x_rot[2], length=100, normalize=True, color='r')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], y_rot[0],
                    y_rot[1], y_rot[2], length=100, normalize=True, color='g')
            self.ax3.quiver(origin_pose[0], origin_pose[1], origin_pose[2], z_rot[0],
                    z_rot[1], z_rot[2], length=100, normalize=True, color='b')
            self.ax3.scatter(xs=[origin_pose[0]], ys=[origin_pose[1]],
                    zs=[origin_pose[2]], marker='o')


    def slider(self):
        fig = plt.figure(figsize=(6, 8))
        self.ax3 = fig.add_subplot(111, projection='3d', position=[0.1, 0.3, 0.8, 0.8])

        self.draw_axes_tf(self.poses)

        # Creacion de sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'Hombro', -180, 180, valinit=self.val_list[0])
        self.slider2 = Slider(self.slider2_ax, 'Codo', -180, 180, valinit=self.val_list[1])
        self.slider3 = Slider(self.slider3_ax, 'Prismático [mm]', -50, 50, valinit=self.val_list[2])
        
        # Agregar boton de reseteo
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        plt.show()


        
    def update(self, val):
        # Limpiar el plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val])
        self.val_list = slider_values
        self.get_pose_to_origin()
        self.draw_axes_tf(self.poses)
        pass

    # Crear un reset para los sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        self.slider4.reset()
        pass
    """
    Función de cinematica directa
    recibe un array de q's que representan las variables que puede tomar el SCARA prismático
    devuelve un array con las coordenadas x,y,z que representa la posición del efector final
    en el espacio cartesiano 3D
    """

    def forward_kinematics(self, parametros):
        """ Funcionamiento:
        Parametros
            parametros: parametros (angulo o distancia) de los joints en grados o mm.
                    numpy array de forma (3)

        Retorna
            pose: posicion del end effector en mm y orientacion en grados, restringido al workspace del robot.
                    numpy array de forma (3)             
        """
        self.val_list = parametros       
        self.get_pose_to_origin()

        XYZ = np.array([self.poses[3][0][3], self.poses[3][1][3], self.poses[3][2][3]], dtype=float) # Reemplazar por columna correspondiente de la matriz de transformacion homogenea (self.poses)
        # Pose final del end effector
        pose = np.array([XYZ[0], XYZ[1], XYZ[2]])
        return pose


####----------COMPLETAR----------####


    def inverse_kinematics(self, target_pos, q0 = [0, 0, 0]): 
        """
            Parametros
                target_pose: posicion del end effector [x, y, z] en mm
                        lista de ints o floats
                q0: valores iniciales de los joints [q1, q2, q3] en grados o mm
                        lista de ints o floats

            Retorna
                q: parametros (angulo o distancia) de los joints en grados o mm.
                        numpy array de forma (3)

        """
        #Función cinemática directa
        def F(theta):
            self.val_list = theta
            self.get_pose_to_origin()
            XYZ = np.array([self.poses[3][0][3], self.poses[3][1][3], self.poses[3][2][3]], dtype=float)
            f1 = XYZ[0]
            f2 = XYZ[1]
            f3 = XYZ[2]
            return np.array([f1, f2, f3])

        #Método numérico
        def NewtonRaphson(f, q0, target_pos, tol=1e-3, max_iter=1000):
            # 1. Punto de entrada el método
            q = q0 
    
            for _ in range(max_iter):
                # 2. Cálcular el jacobiano
                jacobiano_val = jacobian(f, q)


                # 3. Matriz inversa
                inv_jacobiano = np.zeros((3,3))

                # 4. Método Newton Raphson
                if np.abs(np.linalg.det(jacobiano_val)) > tol:
                    q_new = np.zeros(3)

                # Seguro anti divergencia 
                else:
                    q_new = q + np.random.rand(len(q)) # se mueve theta aleatoriamente


                # 5. Condición de convergencia
                if np.linalg.norm(q_new - q) < tol:
                    return q_new 
                q = q_new
            raise ValueError("No se pudo encontrar la solución después de %d iteraciones" % max_iter)

        q = NewtonRaphson(F, q0, target_pos)
        
        return q


#Ejecución del código
if __name__ == "__main__":

    #Configuración de viusalización
    scara = SCARAPrisPoseViewer()
   

    #estimación inicial
    q0 = [0, 0, 0]
    
    #Posición deseada
    target_pos = [0, 0, 0]
    
    #Cinemática inversa
    inicio = time.time()
    ik = scara.inverse_kinematics(target_pos, q0) 
    print('Cinematica inversa:', ik)
    fin = time.time()
    print('Tiempo de ejecución: ', fin-inicio, '[s]' )

    #Cinemática directa
    dk = scara.forward_kinematics(ik)
    print('Cinematica directa:', dk)

    scara.slider()
