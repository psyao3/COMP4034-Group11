#!/usr/bin/env python

import math
import numpy as np


def calculate_linear_distance(pose1, pose2):
    x1, y1 = pose1.x, pose1.y
    x2, y2 = pose2.x, pose2.y
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)


def calculate_angular_distance(pose1, pose2):
    theta1, theta2 = pose1.theta, pose2.theta

    phi = abs(theta2 - theta1) % (2*math.pi)
    if phi > math.pi:
        distance = (2 * math.pi) - phi
    else:
        distance = phi
    return distance


def average_distance(ranges, index, degree):

    # Get range of angles to compute average distance over
    angles = range(index - degree, index + degree - 1)

    # enumerate gives the index and value at the same time.
    # this is to allow wrap around if index + degrees exceeds the array bounds
    for i, ang in enumerate(angles):
        if ang > 360:
            angles[i] = ang - 360
        # If index - degrees is negative, it indexes from the end of the array, e.g
        # -1 is 359, so conversion should not be needed.

    # Convert ranges to np array so list indexing can be used (using angles list)
    arr = np.array(ranges)
    avg = np.mean(arr[angles])

    # Return average distance
    return avg
