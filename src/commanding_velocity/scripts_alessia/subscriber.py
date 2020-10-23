#!/usr/bin/env python

import rospy

def _create_subscriber(name, type, callback):
    return rospy.Subscriber(name, type, callback)