from scipy.spatial.transform import Rotation as r
import numpy as np


class IMU:
    def __init__(self, is_front=True):
        self._is_front = is_front
        self._euler_angle = np.array([0, 0, 0])
        self._rotation = r.from_euler('ZYX', self._euler_angle, degrees=True)
        self._yaw_offset = 0

    def get_euler_angle(self):
        return self._euler_angle

    def get_rotation(self):
        return self._rotation

    def get_yaw_offset(self):
        return self._yaw_offset

    # XXX: Only for raw imu data.
    # If MCU can pass pre-processed data, this must be deleted.
    def set_yaw_offset(self):
        if self._is_front:
            self._yaw_offset = self._euler_angle[0] - 180
        else:
            self._yaw_offset = self._euler_angle[0]
        return

    def set_euler_angle(self, angle):
        self._euler_angle = angle
        self._euler_angle[0] = self._euler_angle[0] - self._yaw_offset
        self._rotation = r.from_euler('ZYX', angle, degrees=True)
