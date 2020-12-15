import numpy as np
from math import sqrt
import rospy
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

## Note: the self.occ_grid used in these functions is reshaped into the 2D array in the update_callback

def get_frontiers(self):
    frontiers = []
    # low is the probability that a cell is occupied
    # We only consider a cell a frontiers if it is not occupied (so below the value of low)
  
    # Iterate through the occupancy grid, and get the value for that index occ_grid[y][x],
    # where y goes through the rows and x goes through the columns.
    for y in range(self.occ_grid_info.height):
        for x in range(self.occ_grid_info.width):
            # If the content of this grid square is unknown, ignore it and continue.
            if self.occ_grid[y][x] == -1:
                continue
            # If the content of this grid square is known and below the defined obstacle probability threshold (defined in __init__)
            # AND it has an unknown neighbour, i.e. an adjacent grid square with a -1 value, then it is a frontier and add it to the list.
            elif self.occ_grid[y][x] < self.occ_threshold and exists_unknown_neighbour(self, x, y):
                frontiers.append((x, y))
    #rospy.loginfo("Length of frontiers: %i" % len(frontiers))

    ## Debug information, if no frontiers are found.
    if len(frontiers) == 0:
        rospy.loginfo("Check occupancy grid. -1:{}, size:{}".format(-1 in self.occ_grid, len(self.occ_grid)))
        rospy.loginfo("Check occupancy grid. 0:{}, unique:{}".format(0 in self.occ_grid, np.unique(self.occ_grid)))
    return frontiers

# Checks neighbours for a given [y][x] occupancy grid position
# Returns True if there is an unknown neighbour (value of -1)
# Returns False otherwise (all neighbours are known).
def exists_unknown_neighbour(self, x, y):

    # Acquire neighbours 
    neighbours = get_neighbours(self, x, y).reshape(3, 3)

    # Iterates through neighbours and returns True if a -1  value is found.
    if -1 in neighbours:
        return True
    else:
        return False


# Gets the neighbours of the given cell, where occ_grid[y][x] in this context is occ_grid[row_number][col_number]
def get_neighbours(self, col_number, row_number):

    # Radius of 1 gets the 3x3 grid of cells centered on col_number, row_number
    radius = 1

    # Perform a list comprehension that iterates [y][x] for all neighbouring grid cells, also including the given cell.
    # The if statement performs a bounds check, so the grid cell is only added to the list if the [y][x] co-ords are valid,
    # otherwise NaN is added. This is converted into a np array and returned.
    return np.array(
        [[self.occ_grid[y][x] if y >= 0 and y < len(self.occ_grid) and x >= 0 and x < len(self.occ_grid[0])
          else np.NaN
          for x in range(col_number - radius, col_number + radius + 1)]
         for y in range(row_number - radius, row_number + radius + 1)]
    )

# Takes the robots current x y WORLD position
# Returns the closest frontiers x y WORLD position
# Conversion between world and grid now done in this function.
def get_closest_frontier(self, world_x, world_y, frontiers):
    
    # Convert world position of robot to grid position
    grid_x, grid_y = world_to_occ_grid(self, self.pose.x, self.pose.y)

    # Calculates the distance (not length of path) between each frontier and the current position (L2 Norm)
    # Sorts the distances in ascending order. Noone understands lambda functions so Ayse please don't ask :).
    sorted_frontiers = sorted(frontiers, key=lambda v: sqrt(pow((v[0] - grid_x), 2) + pow((v[1] - grid_y), 2)))

    # Debug
    # rospy.loginfo("Length of sorted frontiers: %i" % len(sorted_frontiers))

    # Use the first frontier in sorted_frontiers, as it is the closest to the robot.
    target_grid_x, target_grid_y = sorted_frontiers[0]

    # Convert this to world co-ordinates and return them
    return occ_grid_to_world(self, target_grid_x, target_grid_y)


# Converts occupancy grid coordinates to world coordinates
# Used to convert the closest frontier grid cell to the robot to a world coordinate
# we can use as a move_base goal.
def occ_grid_to_world(self, grid_x, grid_y):
    world_x = grid_x * self.occ_grid_info.resolution + self.occ_grid_info.origin.position.x
    world_y = grid_y * self.occ_grid_info.resolution + self.occ_grid_info.origin.position.y
    return world_x, world_y


# Converts world coordinates to occupancy grid coordinates
# Used to obtain the robots grid position so that they can be used to 
# find the closest frontier.
def world_to_occ_grid(self, world_x, world_y):
    # Subtract the known occupancy grid origin, divide by the resolution and round/convert to int (like minitask 4)
    grid_x = int(round((world_x - self.occ_grid_info.origin.position.x) / self.occ_grid_info.resolution))
    grid_y = int(round((world_y - self.occ_grid_info.origin.position.y) / self.occ_grid_info.resolution))
    return grid_x, grid_y


# This was used as the action client server didn't work for some reason originally.
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
        self.goal_publisher.publish(goal)
        rospy.sleep(1)

    while self.goal_status != 3:
        self.goal_publisher.publish(goal)

        pass

    rospy.loginfo("The robot has reached the destination.")
    return True


# Based on minitask 4.
def move_to_target_alt(self, position):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    while not client.wait_for_server(rospy.Duration.from_sec(5.0)):
        rospy.loginfo("Waiting for the move_base action server")

    goal = MoveBaseGoal()

    # important because "map" defines we are using map coordinates
    # and not coordinates relative to the robot
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    #set positions of the goal location
    goal.target_pose.pose.position = position
    goal.target_pose.pose.orientation.x = 0.0
    goal.target_pose.pose.orientation.y = 0.0
    goal.target_pose.pose.orientation.z = 0.0
    goal.target_pose.pose.orientation.w = 1.0

    rospy.loginfo("Sending goal location")
    client.send_goal(goal)

    client.wait_for_result(rospy.Duration(40))

    if (client.get_state() ==  GoalStatus.SUCCEEDED):
        rospy.loginfo("You have reached the destination")
        return True
    else:
        rospy.loginfo("The robot failed to reach the destination or interrupted.")
        return False