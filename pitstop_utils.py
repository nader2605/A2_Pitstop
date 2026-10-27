# Pit Stop helper functions
# 41013 Industrial Robotics - Lab Assignment 2

import numpy as np
from spatialmath import SE3


def back(distance):
    return SE3(0, 0, -distance)


def facing(x, y, target):
    angle = np.arctan2(target[1] - y, target[0] - x)
    return SE3(x, y, 0) * SE3.Rz(angle)


class MotionStopped(Exception):
    pass
