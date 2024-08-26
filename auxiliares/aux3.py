import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

class PoseViewer:

    def __init__(self):
        self.q = [0, 0, 0]
        self.poses = np.array([])
        self.ax3 = None

    def get_pose_to_origin_pose(self):
        # TODO: completar con las matrices de transformacion

        q1, q2, q3 = self.q
        
        #l = 

        origin = np.identity(4) # Origen del sistema de referencia

        #Tab = 

        # Mb = np.linalg.multi_dot([origin, Tab])
        
        self.poses = np.array([origin])

    def traslation (self, x = 0, y= 0, z = 0):
        return np.array([[1, 0, 0, x],
                        [0, 1, 0, y],
                        [0, 0, 1, z],
                        [0, 0, 0, 1]])  

    def rot_x(self, qx):

        return np.array([[1, 0, 0, 0], [0, np.cos(qx), -np.sin(qx), 0], [0, np.sin(qx), np.cos(qx), 0], [0, 0, 0, 1]])

    def rot_y(self, qy):

        return np.array([[np.cos(qy), 0, np.sin(qy), 0], [0, 1, 0, 0], [-np.sin(qy), 0, np.cos(qy), 0], [0, 0, 0, 1]])

    def rot_z(self, qz):

        return np.array([[np.cos(qz), -np.sin(qz), 0, 0], [np.sin(qz), np.cos(qz), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def draw_axes_tf(self, poses, name="", color="k"):
        
        for pose in poses:
            self.ax3.set_xlim(-100, 500)
            self.ax3.set_ylim(-300, 300)
            self.ax3.set_zlim(0, 500)
            self.ax3.set_title("Robot")
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
        self.get_pose_to_origin_pose()
        self.draw_axes_tf(self.poses)
        # Create sliders
        self.slider1_ax = plt.axes([0.2, 0.1, 0.65, 0.03])
        self.slider2_ax = plt.axes([0.2, 0.15, 0.65, 0.03])
        self.slider3_ax = plt.axes([0.2, 0.2, 0.65, 0.03])
        resetax = plt.axes([0.8, 0.025, 0.1, 0.04])

        self.slider1 = Slider(self.slider1_ax, 'q1', -90, 90, valinit=0)
        self.slider2 = Slider(self.slider2_ax, 'q2', 0, 100, valinit=0)
        self.slider3 = Slider(self.slider3_ax, 'q3', 150, 300, valinit=150)
        
        button = plt.Button(resetax, 'Reset', color='white', hovercolor='0.975')
        button.on_clicked(self.reset)
        self.slider1.on_changed(self.update)
        self.slider2.on_changed(self.update)
        self.slider3.on_changed(self.update)
        plt.show()
        
        

    def update(self,val):
        #clean the last plot
        self.ax3.cla()
        slider_values = np.array([self.slider1.val, self.slider2.val, self.slider3.val])
        self.q = slider_values
        self.get_pose_to_origin_pose()
        self.draw_axes_tf(self.poses)
        pass

    #create a reset for sliders
    def reset(self, event):
        self.slider1.reset()
        self.slider2.reset()
        self.slider3.reset()
        pass

if __name__ == "__main__":
     
     robot = PoseViewer()
     robot.slider()
