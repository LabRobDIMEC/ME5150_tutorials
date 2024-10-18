import numpy as np
import cv2

class MoveDrone:
    def __init__(self,
                 pose,
                 vel = 1,
                 sigma_translation = 0.0001,
                 sigma_rotation = 0.0001):

        self.pose = np.array(pose) # [x, y, roll]
        self.vel = vel # velocidad de movimiento
        self.target_pose = np.array(pose) # [x, y, roll]
        self.on_target = True
        self.s_tran = 0.001
        self.s_rot = 0.001

    def is_on_target(self):
        dif = np.array(self.target_pose) - np.array(self.pose)
        if np.linalg.norm(dif) < 0.001:
            self.on_target = True
        else:
            self.on_target = False
        return self.on_target
    
    def update_pose(self):
        dif = np.array(self.target_pose) - np.array(self.pose)
        desv_tran = np.random.normal(0, self.s_tran, 2)
        desv_rot = np.random.normal(0, self.s_rot, 1)
        self.pose += dif * self.vel * 1/240 + np.concatenate((desv_tran, desv_rot))
    
    def set_target_pose(self, pose):
        assert len(pose) == 3, "La pose debe tener 3 elementos, [x, y, roll]"
        self.target_pose = pose

    def move_by_key(self, keys):
        d = 0.0001 # variación de posición
        d_degrees = 0.0001 # variación de ángulo
        if keys.get(119)==1:
            # Go forward, W
            self.target_pose[0] += d * np.cos(self.pose[5])
            self.target_pose[1] += d * np.sin(self.pose[5])
        elif keys.get(115)==1:
            # Go back, S
            self.target_pose[0] -= d * np.cos(self.pose[5])
            self.target_pose[1] -= d * np.sin(self.pose[5])
        elif keys.get(97)==1:
            # Go left, A
            self.target_pose[1] += d * np.cos(self.pose[5])
            self.target_pose[0] -= d * np.sin(self.pose[5])
        elif keys.get(100)==1:
            # Go right, D
            self.target_pose[1] -= d * np.cos(self.pose[5])
            self.target_pose[0] += d * np.sin(self.pose[5])
        elif keys.get(113)==1:
            # Go counterclockwise, Q
            self.target_pose[5] += d_degrees
        elif keys.get(101)==1:
            # Go clockwise, E
            self.target_pose[5] -= d_degrees
