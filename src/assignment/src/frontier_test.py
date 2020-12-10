#!/usr/bin/env python

import rospy
from math import sqrt, pow
# import Graph
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from nav_msgs.msg import OccupancyGrid
from map_msgs.msg import OccupancyGridUpdate
from geometry_msgs.msg import Twist, Pose2D, Point, PoseStamped
from nav_msgs.msg import Odometry

# from callbacks import  *
from actionlib_msgs.msg import *
import actionlib
import numpy as np
import grid_methods as grid
import tf


class Tour:

    def __init__(self):

        rospy.sleep(1)

        graph = { "a" : ["b"],
                  "b" : ["a","c"],
                  "c" : ["b","d"],
                  "d" : ["c", "e", "k"],
                  "e" : ["d", "f", "n"],
                  "f" : ["e", "g"],
                  "g" : ["f", "h", "j"],
                  "h" : ["g", "i"],
                  "i" : ["h", "j"],
                  "j" : ["g", "i"],
                  "k" : ["d", "l"],
                  "l" : ["k", "m"],
                  "m" : ["l", "n"],
                  "n" : ["m", "e", "o"],
                  "o" : ["n", "p", "q"],
                  "p" : ["o", "q"],
                  "q" : ["o", "p", "r"],
                  "r" : ["q"]
                }

        node_locations = {
                  "a" : Point(-1.1, 3.8, 0),
                  "b" : Point(-1.1, 1.36, 0),
                  "c" : Point(-1.1, -1.22, 0),
                  "d" : Point(-0.01, -0.34, 0),
                  "e" : Point(0.07, 0.98, 0),
                  "f" : Point(0.18, 4.17, 0),
                  "g" : Point(1.62, 4.11, 0),
                  "h" : Point(4.14, 3.93, 0),
                  "i" : Point(3.82, 2.32, 0),
                  "j" : Point(1.45, 2.5, 0),
                  "k" : Point(0.77, -1.63, 0),
                  "l" : Point(0.612, -3.53, 0),
                  "m" : Point(3.07, -3.6, 0),
                  "n" : Point(2.66, 1.04, 0),
                  "o" : Point(4.65, 0.88, 0),
                  "p" : Point(5.77, 3.51, 0),
                  "q" : Point(5.77, 0.78, 0),
                  "r" : Point(5.77, -2.55, 0)
        }

        self.visited_nodes = []
        self.unvisited_nodes = node_locations.copy()
        self.occ_grid = None
        self.occ_grid_info = None
        self.pose = Pose2D()

        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.status_callback)
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, self.grid_callback)
        while self.occ_grid_info is None:
            pass
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, self.update_callback)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.goal_status = -1

        frontiers = self.get_frontiers()
        #print(frontiers)
        print("Pose")
        print(self.pose)
        print("Grid coordinates")
        print(self.world_to_occ_grid(self.pose.x, self.pose.y))
        print("Closest frontier")
        print(self.get_closest_frontier(self.pose.x, self.pose.y, frontiers))
        print(self.print_grid())

    def get_frontiers(self):
        frontiers = []
        low = 20  # Need to change this to something that makes sense
        for y in range(self.occ_grid_info.height):
            for x in range(self.occ_grid_info.height):
                if self.get_occupancy_grid_value(x, y) == -1:
                    continue
                if self.get_occupancy_grid_value(x, y) < low and self.exists_unknown_neighbour(x, y):
                    frontiers.append((x, y))
        return frontiers

    def print_grid(self):
        # To scale it down to 20x20 I need a 400x400 matrix
        # So I'm padding our grid (384x384) to be 400x400
        # Then I'm trying to take the mean for every 20x20 subgrid
        # But I can't figure out which axis I need to use. lol
        occ_grid = self.occ_grid
        padded_grid = np.ones((400,400))
        padded_grid[:occ_grid.shape[0],:occ_grid.shape[1]] = occ_grid
        resized_grid = padded_grid.reshape(20,20,20,20).mean(-1).mean(1)

        # Print 1d grid
        print "\n\n"

        for line in resized_grid:
            values = [' - ' if val == -1 else ' X ' for val in line]
            values = [str(int(val)) for val in line]
            print ' '.join(values)

        print("\n\n")

    def exists_unknown_neighbour(self, x, y):
        # Remove current cell from neighbours
        neighbours = self.get_neighbours(x, y).reshape(3, 3)
        neighbours[1,1] = 0
        if -1 in neighbours:
            return True
        else:
            return False

    def get_neighbours(self, row_number, col_number):
        radius = 1
        return np.array(
            [[self.occ_grid[row][col] if row >= 0 and row < len(self.occ_grid) and col >= 0 and col < len(self.occ_grid[0])
              else np.NaN
              for col in range(col_number - 1 - radius, col_number + radius)]
             for row in range(row_number - 1 - radius, row_number + radius)]
        )

    def get_closest_frontier(self, current_x, current_y, frontiers):
        current_pos = (current_x, current_y)
        sorted_frontiers = sorted(frontiers, key=lambda v: sqrt(pow((v[0] - current_pos[0]), 2) + pow((v[1] - current_pos[1]), 2)))
        return sorted_frontiers[0]

    def status_callback(self, msg):
        self.goal_status = msg.status.status

    def grid_callback(self, msg):
        self.occ_grid = np.array([-1 for cell in msg.data], dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occ_grid_info = msg.info

    def update_callback(self, msg):
        update = np.array(msg.data, dtype=np.int8).reshape(msg.height, msg.width)
        self.occ_grid[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = update
        #self.print_grid()

    def odom_callback(self, msg):
        # Get (x, y, theta) specification from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y, \
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)

        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

    def get_occupancy_grid_value(self, x, y):
        return self.occ_grid[y][x]

    def occ_grid_to_world(self, grid_x, grid_y):
        world_x = grid_x * self.occ_grid_info.resolution + self.occ_grid_info.origin.position.x
        world_y = grid_y * self.occ_grid_info.resolution + self.occ_grid_info.origin.position.y
        return world_x, world_y

    def world_to_occ_grid(self, world_x, world_y):
        grid_x = int(round((world_x - self.occ_grid_info.origin.position.x) / self.occ_grid_info.resolution))
        grid_y = int(round((world_y - self.occ_grid_info.origin.position.y) / self.occ_grid_info.resolution))
        return grid_x, grid_y

    def patrol(self):
        rospy.loginfo("Start patrol.")
        # Empty dictionaries {} evaluate to False
        while len(self.unvisited_nodes) != 0:
            # Get next node/key
            node = list(self.unvisited_nodes)[0]
            target = self.unvisited_nodes.pop(node)
            rospy.loginfo("Target: {}".format(target))

            # Need to navigate to the target via defined edges
            self.move_to_target(target)

            self.visited_nodes.append(target)

    # Take the start/end node and existing path. If this is the first
    # function call, then path = []
    def find_path(self, graph, start_node, end_node, path):
        # Add the current start node to the path list.
        path.append(start_node)

        # Destination is reached, so the entire path is found
        if start_node == end_node:
            return path

    def move_to_target(self, position):

        goal = PoseStamped()

        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"

        goal.pose.position.x = position.x
        goal.pose.position.y = position.y
        goal.pose.position.z = 0.0

        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        while self.goal_status != 1 and self.goal_status != 2:
            rospy.loginfo("Waiting for goal to be accepted.")
            rospy.loginfo(self.goal_status)
            self.goal_publisher.publish(goal)
            rospy.sleep(1)

        while self.goal_status != 3:
            pass

        rospy.loginfo("The robot has reached the destination.")
        return True


if __name__ == '__main__':
    try:
        rospy.init_node('tour')
        tour = Tour()
        tour.patrol()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e
    except KeyboardInterrupt:
        print("Program terminated.")
        exit()

