from scipy.spatial.transform import Rotation as r
import numpy as np


class IMU:
    def __init__(self):
        self._euler_angle = np.array([0, 0, 0])
        self._rotation = r.from_euler('xyz', self._euler_angle)

    def get_euler_angle(self):
        return self._euler_angle

    def get_rotation(self):
        return self._rotation

    def set_euler_angle(self, angle):
        self._euler_angle = angle + 0
        self._rotation = r.from_euler('xyz', angle)
