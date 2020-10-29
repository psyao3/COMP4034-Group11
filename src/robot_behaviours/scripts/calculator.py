#!/usr/bin/env python

import math
from geometry_msgs.msg import Pose2D


def calculate_linear_distance(pose1, pose2):
    x1, y1 = pose1.x, pose1.y
    x2, y2 = pose2.x, pose2.y
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

'''
def calculate_angular_distance(pose1, pose2):
    theta1, theta2 = pose1.theta, pose2.theta
    angle = theta2 - theta1

    if angle > math.pi:
        angle -= 2 * math.pi
    elif angle < -math.pi:
        angle += 2 * math.pi
    return abs(angle)


def calculate_angular_distance(pose1, pose2):
    theta1, theta2 = pose1.theta, pose2.theta

    if theta1 < 0:
        theta1 += 2 * math.pi
    if theta2 < 0:
        theta2 += 2 * math.pi
    print(theta1, theta2, abs(theta2 - theta1))
    return abs(theta2-theta1)
'''


def calculate_angular_distance(pose1, pose2):
    theta1, theta2 = pose1.theta, pose2.theta

    phi = abs(theta2 - theta1) % (2*math.pi)
    if phi > math.pi:
        distance = (2 * math.pi) - phi
    else:
        distance = phi
    return distance
