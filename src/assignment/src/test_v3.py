#!/usr/bin/env python

import actionlib

import math
import random
import rospy
import struct
import time

from actionlib_msgs.msg import *
from darknet_ros_msgs.msg import BoundingBoxes, ObjectCount
from geometry_msgs.msg import Point, Pose2D, PoseStamped, Twist
from map_msgs.msg import OccupancyGridUpdate

from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan

import numpy as np
from matplotlib import pyplot as plt


from callbacks import *
from frontier_methods import *



class Follower:

    def __init__(self):

        # Initialising rospy behaviournp
        rospy.on_shutdown(self.stop)
        self.rate = rospy.Rate(10)  # Hz

        # Obstacle Avoidance thresholds
        self.front_thresh = 0.5
        self.side_thresh = 0.3

        # Speeds to use
        self.linear_speed = 0.3
        self.angular_speed = math.pi/8

        # Occupancy grid initialisation
        self.occ_grid = None
        self.occ_grid_info = None
        self.occ_threshold = 50 # Threshold for obstacle probability; so that frontiers are only considered if below this value.
        
        # Pose for odom
        self.pose = Pose2D()

        # Initialise ranges
        self.ranges = None
        self.closest_obstacle = None
        self.right_obst, self.front_obst, self.left_obst = None, None, None
        self.any_obst = None

        # Bounding box related variables
        self.goal_x = 0
        self.goal_y = 0
    
        self.current_tgt_class = None
        self.is_target = False

        # Keep track of targets that have been reached.
        self.complete = []

        # List of frontiers to ignore
        self.ignore_frontiers = []

        # For number 5 special case
        self.box_height = None 
        self.box_width = None


        # Variables for evaluation
        self.start_time = time.time()
        self.time_limit = 300 # 5 minutes

        # Initialise move_base goal status
        self.goal_status = -1

        # Publishers and Subscribers
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.occ_grid_publisher = rospy.Publisher('/occ_grid', Float32MultiArray, queue_size=10)

        # Darknet
        # This only updates when bounding boxes are found, so need another subscriber.
        self.box_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, box_callback, (self))
        # Updates constantly with number of bounding boxes
        self.count_obj_sub = rospy.Subscriber('/darknet_ros/found_object', ObjectCount, count_boxes_callback, (self) )
        self.box_count = 0 # Initialise


        self.odom_sub = rospy.Subscriber('/odom', Odometry, odom_callback, (self))
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, scan_callback, (self))

        self.frontier_pub = rospy.Publisher('/frontier', OccupancyGrid, queue_size=1)

        # Subsribers for occupancy grids - global costmap and updates
        # self.occ_grid_ingo needs to be initialised before it's updated
        rospy.Subscriber('/move_base/global_costmap/costmap', OccupancyGrid, grid_callback, (self))
        while self.occ_grid_info is None:
            rospy.loginfo("Waiting for occupancy to be initialised.")
            rospy.sleep(0.5)
        rospy.Subscriber('/move_base/global_costmap/costmap_updates', OccupancyGridUpdate, update_callback, (self))

        # Publisher to cancel current move base command to use in box callback.
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        # Start main control function
        self.main()


    # Publish a stop message
    def stop(self):
        self.pub.publish(Twist())


   # Check if target reached
    # IF the bottom of the bounding box (goal_y) is close to the bottom
    # of the camera view in pixels (>1080 * 0.8), then object reached.
    #self.explore()
    # FOR NUMBER 5 ONLY:
    # If the box width is 1.5 * box height, goal reached. May need to tweak
    # this, but chose this based on observed box behaviour when closing in on
    # the number 5 object.
    def check_arrival(self):
        
        # Define h,w of image input
        w = 1920
        h = 1080
        if self.is_target:
            mailbox = (self.current_tgt_class == "mailbox legs") and (self.goal_y >= h * 0.75)
            fire = (self.current_tgt_class == "fire hydrant") and (self.box_width > 1.2 * self.box_height)
            box = (self.current_tgt_class == "green box") and (self.goal_y >= h * 0.75)
            five = (self.current_tgt_class == "number 5") and  (self.box_width > 1.5 * self.box_height)
        else:
            # Can't possibly be a target
            mailbox = fire = box = five = False

        return mailbox or fire or box or five

    def main(self):

        # Main control here. Control logic extracted from callbacks.
    
       

        # Stop if all 4 targets have been found. TODO: add condition to stop after 5 mins (demo timer)
        while not rospy.is_shutdown() and not len(self.complete) > 3: #and (rospy.get_rostime() - self.start_time) <= self.time_limit:
            
            #if time.time() - self.start_time > self.time_limit:
                #rospy.loginfo("TIME LIMIT EXPIRED")
          
            # Is true if we have arrived at an obstacle
            if self.check_arrival():
                # Stop
                self.stop()

                # Add target to list of those already found
                self.complete.append(self.current_tgt_class)
                self.is_target = False

                # Log info
                rospy.loginfo(self.current_tgt_class + " target reached.")
                rospy.loginfo("Stopping.")
                rospy.loginfo(self.complete) # Print obstacles found so far

                # Sleep so it doesn't beacon to another target immediately (noticed with fire extinguisher/mailbox)
                rospy.sleep(1.5)

                # Explore immediately after, instead of obstacle avoidance
                if len(self.complete) > 3:
                    break
                else:
                    self.explore()

            # If theres an obstacle closer than threshold (0.5) do obstacle avoidance
            elif self.closest_obstacle <= self.side_thresh or self.left_obst <= self.side_thresh or self.front_obst <= self.front_thresh:
                self.obstacle_avoidance() 
                rospy.sleep(.25)

              # Beacon towards target
            elif self.is_target:
                self.beacon_to_target()
                rospy.sleep(0.25)   

            else:
                # Frontier exploration.
                self.explore()
            
            
            # Sleep
            self.rate.sleep()

        # We did it! All targets reached and acknowledged :)
        rospy.loginfo("All targets reached, shutting down.")
        self.evaluate()



    # Code to perform frontier exploration algorithm. Gets the frontiers, then gets the closest,
    # and then performs a move_base command to move to it. Gets interrupted if a target is spotted
    # by the box_callback.
    def explore(self):

        '''
        Frontier Exploration logic
        '''
        rospy.loginfo("Start exploration.")

        while self.pose is None:
            rospy.loginfo("Waiting for pose to be initialised.")

        # Frontiers include the ones outside the arena
        frontiers = get_frontiers(self)

        if len(frontiers) == 0:
            rospy.loginfo("Clearing ignored frontiers")
            self.ignore_frontiers = []
            frontiers = get_frontiers(self)
       
        while len(frontiers) != 0 and not self.is_target:
           
            # Visualise frontiers
            # self.visualize_frontiers(frontiers)
            self.publish_frontiers(frontiers)

            # Get closest frontiers (point distance - not path)
            target_x, target_y = get_closest_frontier(self, self.pose.x, self.pose.y, frontiers)
           
            # Move to target
            target_point = Point(target_x, target_y, 0)
            #rospy.loginfo("Target: {}".format(target_point))

            if not move_to_target_alt(self, target_point):
                # Assume the frontier is unreachable and ignore it in future.
                rospy.loginfo("Could not reach frontier - ignoring this frontier.")
                # TODO: figure this out
                # Find frontiers adjacent to the target point
                grid_x, grid_y = world_to_occ_grid(self, target_x, target_y)
                update_unreachable_frontiers(self, (grid_x, grid_y), frontiers)
                


            # Frontiers include the ones outside the arena
            frontiers = get_frontiers(self)

            # Drop current ignored frontiers

            #self.ignore_frontiers = []
         

   
    # Function to display frontiers in maplotlib plot.
    # No clue whats going on here......
    def visualize_frontiers(self, frontiers):
        # Initialise a 384x384 grid full of ones
        grid_target = np.ones((384, 384))
        # For each point in the frontiers change the value of grid_target to 0
        for x, y in frontiers:
            grid_target[y][x] = 0
        # Rotate the grid by 90 degrees
        grid_target = [[grid_target[y][x] for y in range(len(grid_target))] for x in
                        range(len(grid_target[0]) - 1, -1, -1)]
        # Flip the grid along the vertical axis to match rviz
        grid_target = np.flip(grid_target, 1)
        # Plot Image
        plt.imshow(grid_target, cmap='hot', interpolation='nearest')
        plt.show()

    # Publish the frontiers to a topic so they can be visualized in RViz
    def publish_frontiers(self, frontiers):
        # Initialise a 384x384 grid full of ones
        grid_target = np.zeros((384, 384))
        # For each point in the frontiers change the value of grid_target to 0
        for x, y in frontiers:
            grid_target[y][x] = 100
        # Publish the frontiers
        frontiers_msg = OccupancyGrid()
        frontiers_msg.info = self.occ_grid_info
        frontiers_msg.data = np.asarray(grid_target).reshape(-1).astype(int)
        self.frontier_pub.publish(frontiers_msg)

    # For printing stats at the end of a run for evaluation.
    def evaluate(self):

        run_time = time.time() - self.start_time

        rospy.loginfo("Statistics of this run:")
        rospy.loginfo("Runtime in minutes: %f" % run_time/60)
        rospy.loginfo("Targets found: %d" % len(self.complete))
        rospy.loginfo(self.complete) # The targets

        # Extra stats can be added w.r.t to frontiers
        not_visited = np.count_nonzero(self.occ_grid.reshape(-1) == -1)
        not_visited_prc = not_visited/float(len(self.occ_grid.reshape(-1))) * 100

        self.ignore_frontiers = []
        frontiers = get_frontiers(self)

        rospy.loginfo("Number of known frontiers remaining (individual points): %i" % len(frontiers))
        rospy.loginfo("Number of unknown cells remaining: %f" % not_visited)
        rospy.loginfo("Percentage of unknown cells remaining: %f %" % not_visited_prc)


    # Adapted obstacle avoidance from previous tasks.
    def obstacle_avoidance(self):
     
        twist_msg = Twist()
        # If obstacle both sides, turn 90 degrees to try and clear both.
        if self.left_obst <= self.side_thresh and self.right_obst <= self.side_thresh:
            if self.front_obst <= self.front_thresh:
                rospy.loginfo("Obstacle left and right; Turning 90 degrees")
                twist_msg.angular.z = -math.pi/3
                self.pub.publish(twist_msg)
                rospy.sleep(1.5)
                return
            else:
                twist_msg.linear.x = self.linear_speed
        
        # Turn away from a left obstacle
        elif self.left_obst <= self.side_thresh:
            rospy.loginfo("Obstacle left; Turning clockwise")
            twist_msg.angular.z = -self.angular_speed
        # Turn away from a right obstacle
        elif self.right_obst <= self.side_thresh:
            rospy.loginfo("Obstacle right; Turning anticlockwise")
            twist_msg.angular.z = self.angular_speed
        # Turn way from an obstacle directly in front.
        elif self.front_obst <=  self.front_thresh:
            rospy.loginfo("Obstacle ahead; Turning clockwise")
            twist_msg.angular.z = -self.angular_speed

        self.pub.publish(twist_msg)
        
        # Get x centroid of bounding box of target
    def beacon_to_target(self):
        
        # Get x centroid of bounding box of target
        box_centroid = self.goal_x

        # And width of the camera image (provided by darknet_ros)
        # it seems to be 1920x1080, so im hard coding it for now.
        w = 1920

        # Calculate the error for prop. control
        err = box_centroid - w/2

        # Construct twist message and publish it
        twist_msg = Twist()
        #twist_msg.linear.x = self.linear_speed
        twist_msg.linear.x = self.linear_speed #min(self.linear_speed * self.closest_obstacle, self.linear_speed)
        twist_msg.angular.z = -float(err) / 400

        rospy.loginfo("Beaconing towards " + self.current_tgt_class)
        self.pub.publish(twist_msg)



if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        rospy.spin()
    except rospy.ROSInterruptException as e:
        print("An exception was caught.")
        raise e

