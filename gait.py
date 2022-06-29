import csv, math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from scipy.spatial.transform import Rotation as R
from imu import IMU


LEFT = 0
RIGHT = 1
FOOT = 20
SHANK = 50
THIGH = 50
TORSO = 20

index = 0

LIMB_LENGTH = [THIGH, SHANK, FOOT, TORSO]

fig, ax = plt.subplots()
ax.set_xlim(-100, 100)
ax.set_ylim(-100, 100)

x, y = [], []
line, = plt.plot([], [], 'bo')

class AFO:
    def __init__(self, limb_length=(FOOT, SHANK, THIGH, TORSO)):
        self._sensor = [IMU()] * 7
        self._current_stance_foot = None
        self._joint_position = np.zeros([8, 2])
        self._limb_length = np.array(limb_length)
        self.__joint_order = [
            {0: [0, 0], 1:[1, 3], 2:[2, 2], 3:[3, 1], 4: [4, 4], 5:[5, 5], 6:[6,6]},
            {0:[6, 0], 1:[5, 6], 2:[4, 5], 3:[3, 4], 4:[2, 1], 5:[1, 2], 6:[0, 3]}
        ]

    def get_joint_position(self):
        return self._joint_position

    def get_limb_length(self):
        return self._limb_length

    def set_limb_length(self, limb_length: list):
        self._limb_length = np.array(limb_length)
        return

    def update_sensor(self, angle):
        for i in range(7):
            self._sensor[i].set_euler_angle(angle[i])

    """
    joint number order
        0 - Left toe        1 - Left ankle        2 - Left knee
        3 - Left torso      4 - Right torso       5 - Right knee
        6 - Right ankle     7 - Right toe
    
    IMU number order
        0 - Left foot       1 - Left shank        2 - Left thigh
        3 - Torso           4 - Right foot        5 - Right shank
        6 - Right thigh
    """
    def update_afo(self):
        initial_z_vector = np.array([0, 0, 1.])
        self._joint_position[0] = [0, 0, 0]
        for i in range(1, 8):
            # Determine which type of limb for current index.
            if i == 1 or i == 7:
                current_limb_length = self._limb_length[0]
            elif i == 2 or i == 6:
                current_limb_length = self._limb_length[1]
            elif i == 3 or i == 5:
                current_limb_length = self._limb_length[2]
            else:
                current_limb_length = self._limb_length[3]

            self._joint_position[i] = \
                self._joint_position[i - 1] \
                + self._sensor[i - 1].get_rotation().apply(initial_z_vector) \
                * current_limb_length

        lowest_height = min(self._joint_position.transpose()[2])
        for i in range(8):
            self._joint_position[i][2] -= lowest_height

    def switch_stance(self):
        pass


def visualizer(frame, afo: AFO, data):
    global index
    new_angle = np.array([7, 3])
    for i in range(7):
        new_angle[i] = data[i][index]
    index += 1
    afo.update_sensor(new_angle)
    afo.update_afo()
    current_position = afo.get_joint_position()
    x = current_position.transpose()[0]
    y = current_position.transpose()[1]
    line.set_data(x, y)
    return line,


def main():
    data = [np.array([])] * 7

    with open('../data/Two_heel.csv', newline='') as csvfile:
        reader = csv.reader(csvfile, delimiter=',')
        for row in reader:
            current_index = int(row[0])
            if current_index < 7:
                current_data = [float(row[1]), float(row[2]), float(row[3])]
                data[current_index] = \
                    np.append(data[current_index], current_data)

    for i in range(7):
        data[i] = data[i].reshape(
            [len(data[i])//3, 3])

    afo = AFO()

    ani = FuncAnimation(fig, visualizer, frames=200, fargs=(afo, data))




    return


if __name__ == "__main__":
    main()