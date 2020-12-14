import numpy as np
from math import sqrt
import rospy
from geometry_msgs.msg import PoseStamped


def get_frontiers(self):
    frontiers = []
    low = 70
    for y in range(self.occ_grid_info.height):
        for x in range(self.occ_grid_info.width):
            if get_occupancy_grid_value(self, x, y) == -1:
                continue
            elif get_occupancy_grid_value(self, x, y) < low and exists_unknown_neighbour(self, x, y):
                #rospy.loginfo(get_neighbours(self, x, y))
                frontiers.append((x, y))
                #rospy.sleep(0.3)
    return frontiers


def exists_unknown_neighbour(self, x, y):
    neighbours = get_neighbours(self, x, y).reshape(3, 3)
    if -1 in neighbours:
        return True
    else:
        return False


def get_neighbours(self, col_number, row_number):
    # Get the 9 cells surrounding and including the current one
    radius = 1
    return np.array(
        [[self.occ_grid[row][col] if row >= 0 and row < len(self.occ_grid) and col >= 0 and col < len(self.occ_grid[0])
          else np.NaN
          for col in range(col_number - radius, col_number + radius + 1)]
         for row in range(row_number - radius, row_number + radius + 1)]
    )


def get_closest_frontier(self, current_x, current_y, frontiers):
    current_pos = (current_x, current_y)
    sorted_frontiers = sorted(frontiers, key=lambda v: sqrt(pow((v[0] - current_pos[0]), 2) + pow((v[1] - current_pos[1]), 2)))
    return sorted_frontiers[0]


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
        pass

    rospy.loginfo("The robot has reached the destination.")
    return True