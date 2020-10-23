#!/usr/bin/env python

import rospy

def _create_publisher(name, type, size_of_queue = 10):
    return rospy.Publisher(name, type, queue_size=size_of_queue)