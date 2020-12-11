#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from math import sqrt


class MapListener:

    def __init__(self):
        rospy.init_node('map_listener', anonymous=True)
        rospy.Subscriber('/occ_grid', Float32MultiArray, self.grid_callback)
        rospy.spin()

    def grid_callback(self, msg):
        print('Received grid.')
        length = len(msg.data)
        sqrt_len = sqrt(length)
        self.print_grid(np.asarray(msg.data).reshape((sqrt_len, sqrt_len)))

    def print_grid(self, occ_grid):
        # To scale it down to 20x20 I need a 400x400 matrix
        # So I'm padding our grid (384x384) to be 400x400
        # Then I'm trying to take the mean for every 20x20 subgrid
        # But I can't figure out which axis I need to use. lol
        padded_grid = np.ones((400,400))
        padded_grid[:occ_grid.shape[0],:occ_grid.shape[1]] = occ_grid
        resized_grid = padded_grid.reshape(20,20,20,20).mean(-1).mean(1)

        # Print 1d grid
        print "\n\n"

        for line in resized_grid:
            values = [' - ' if val == -1 else ' X ' for val in line]
            #values = [str(int(val)) for val in line]
            print ' '.join(values)

        print("\n\n")


if __name__ == '__main__':
    MapListener()