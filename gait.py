import csv, math, time
import numpy as np
import threading, serial
import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from imu import IMU


LEFT = 0
RIGHT = 1
FOOT = 20
SHANK = 50
THIGH = 50
TORSO = 20
LIMB_LENGTH = [THIGH, SHANK, FOOT, TORSO]


class AFO:
    def __init__(self, limb_length=(FOOT, SHANK, THIGH, TORSO)):
        self._sensor = []
        for i in range(7):
            if i in [0, 3, 6]:
                self._sensor.append(IMU(False))
            else:
                self._sensor.append(IMU())

        self._yaw_offset = False
        self._toe_clearance = 0
        self._joint_position = np.zeros([8, 3])
        self._limb_length = np.array(limb_length)

    def get_sensor(self, num):
        return self._sensor[num]

    def get_joint_position(self):
        return self._joint_position

    def get_limb_length(self):
        return self._limb_length

    def get_yaw_offset(self):
        return self._yaw_offset

    def set_limb_length(self, limb_length: list):
        self._limb_length = np.array(limb_length)
        return

    def update_sensor(self, imu_num, angle):
        self._sensor[imu_num].set_euler_angle(angle)
        return

    # XXX: Only for raw imu data.
    # If MCU can pass pre-processed data, this must be deleted.
    def update_yaw_offset(self):
        self._yaw_offset = True
        for i in range(7):
            self._sensor[i].set_yaw_offset()

    """
    joint number order
        0 - Left toe        1 - Left ankle        2 - Left knee
        3 - Left torso      4 - Right torso       5 - Right knee
        6 - Right ankle     7 - Right toe
    
    IMU number order
        0 - Left foot       1 - Left shank        2 - Left thigh
        3 - Torso           4 - Right thigh        5 - Right shank
        6 - Right foot
    """
    def update_afo(self):
        zero_vector = np.array([[0, 0, 1],
                                [0, -1, 0],
                                [0, -1, 0],
                                [-1, 0, 0],
                                [0, 1, 0],
                                [0, 1, 0],
                                [0, 0, -1]
                                ])
        self._joint_position[0] = [0, 0, 0]
        for i in range(1, 8):
            # Determine which type of limb for current index.
            if i == 1 or i == 7:
                current_limb_type = 0
            elif i == 2 or i == 6:
                current_limb_type = 1
            elif i == 3 or i == 5:
                current_limb_type = 2
            else:
                current_limb_type = 3

            current_limb_length = self._limb_length[current_limb_type]

            # Calculate each joint position based on left foot position.
            self._joint_position[i] = \
                self._joint_position[i - 1] \
                + self._sensor[i - 1].get_rotation().apply(zero_vector[i - 1]) \
                * current_limb_length

        # To maintain foot ground contact,
        # calculate lowest z-value and use it as an offset for z-value.
        lowest_height = min(self._joint_position.transpose()[2])

        # To maintain limb's position at center position of graph,
        # get left torso x, y value and use them as offsets for x, y values.
        torso_position = self._joint_position[3][0:2].copy()

        # Update joint position with above offsets.
        for i in range(8):
            self._joint_position[i][2] -= lowest_height
            self._joint_position[i][0:2] -= torso_position

        self._toe_clearance = max(self._joint_position[0][2],
                                  self._joint_position[7][2])

        # Print toe clearance instead of visualizing it on this version.
        # Please use this value for your own visualization program.
        print(self._toe_clearance)


# --------------------------------------------------------------------#
# For IMU data receiving.
# They are temporary version to record sample video.
# Line # : 122 -184
class ReadLine:
    def __init__(self, s):
        self.buf = bytearray()
        self.s = s

    def read_line(self):
        i = self.buf.find(b'\n')
        if i >= 0:
            r = self.buf[:i + 1]
            self.buf = self.buf[i + 1:]
            return r
        while True:
            i = max(1, min(2048, self.s.in_waiting))
            data = self.s.read(i)
            i = data.find(b'\n')

            if i >= 0:
                r = self.buf + data[:i + 1]
                self.buf[0:] = data[i + 1:]
                return r
            else:
                self.buf.extend(data)


def raw_parsing(_raw):
    _parsed = _raw.decode().split(',')
    return _parsed


def ser_init():
    _ser = serial.Serial(port='COM4', baudrate=921600)
    _ser.close()
    print("serial closed")
    time.sleep(0.3)
    _ser.open()
    print("serial opened")
    time.sleep(0.3)
    return _ser


def update_sensor(afo):
    ser = ser_init()
    rl = ReadLine(ser)
    time.sleep(1)
    for i in range(100000000):
        res = raw_parsing(rl.read_line())
        if len(res) < 5:
            continue
        imu_num = int(res[0][-1])
        if imu_num < 0 or imu_num > 6:
            continue
        new_angle = np.array((float(res[3]), float(res[2]), float(res[1])))
        afo.update_sensor(imu_num, new_angle)
        if i == 100 and not afo.get_yaw_offset():
            afo.update_yaw_offset()


# END of IMU data receiving program.
# --------------------------------------------------------------------#


def visualizer(frame, afo: AFO, line, line2):
    afo.update_afo()
    current_position = afo.get_joint_position().transpose()
    line.set_data(
        current_position[0:2],
    )
    line.set_3d_properties(current_position[2])
    line2.set_data(
        current_position[0:2]
    )
    line2.set_3d_properties(current_position[2])
    return line, line2


def main():
    afo = AFO()
    fig = plt.figure()
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_proj_type('persp')
    ax1.view_init(azim=270, elev=5)
    ax2 = fig.add_subplot(122, projection='3d')
    ax2.set_proj_type('ortho')
    ax2.view_init(azim=180, elev=0)

    line, = ax1.plot([], [], [], 'bo-')
    line2, = ax2.plot([], [], [], 'ro-')
    ax1.set_xlim(-100, 100)
    ax1.set_xlabel("x")
    ax1.set_ylabel("y")
    ax1.set_ylim(-100, 100)
    ax1.set_zlim3d(-2, 150)
    ax2.set_xlim(-100, 100)
    ax2.set_xlabel("x")
    ax2.set_ylabel("y")
    ax2.set_ylim(-100, 100)
    ax2.set_zlim3d(-2, 150)

    t = threading.Thread(target=update_sensor, args=[afo])
    t.start()

    ani = FuncAnimation(fig, visualizer, interval=1, frames=60, fargs=(afo, line, line2))

    plt.show()

    return


if __name__ == "__main__":
    main()