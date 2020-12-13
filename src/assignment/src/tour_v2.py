#!/usr/bin/env python

import rospy
# import Graph
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

from geometry_msgs.msg import Twist, Pose2D, Point, PoseStamped
from nav_msgs.msg import Odometry
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import Image, CameraInfo

# from callbacks import  *
from actionlib_msgs.msg import *
import actionlib
import cv2, cv_bridge
import image_geometry as img_geo


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

        self.object_locations = []
        self.complete = []

        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        rospy.Subscriber('move_base/result', MoveBaseActionResult, self.status_callback)

        self.goal_status = -1
        self.is_target = False
        self.current_tgt_class = None

        # Initialise image processing
        self.bridge = cv_bridge.CvBridge()

        self.bbox_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.box_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/image', Image, self.depth_callback)
        

        self.goal_x = float(0)
        self.goal_y = float(0)
        self.depth = float(0)

    def depth_callback(self, data):
        '''
        centralises the bounding box on the robot vision and creates a move base goal 
        '''

        depth_image = self.bridge.imgmsg_to_cv2(data, "passthrough")

        # distance to object
        Z = depth_image[self.goal_y,self.goal_x]

        self.depth = Z
        
    


    def box_callback(self, data):

        # We need to ignore targets or boxes belonging to a class
        # we've already found.
        ignore = self.complete

        # If no boxes set values and return early.
        # Check each bounding box found, ignore if already reached that class
        boxes = []

        for box in data.bounding_boxes:
            if box.Class not in ignore:
                boxes.append(box)

        # If boxes is still empty, it means no new targets found,
        # set variables and return early. (empty list is false)
        if not boxes:
            self.is_target = False
            self.current_tgt_class = None
            return


        # Break after variables set.
        for box in data.bounding_boxes:
        
            # Get the class
            obj_class = box.Class
            
            # Get the centers of the bounding box (x is needed for prop controller)
            x = ((box.xmax - box.xmin)/2)+box.xmin
            y = ((box.ymax - box.ymin)/2)+box.ymin

            # x for beaconing
            self.goal_x = float(x)

            # y for identifying when we've reached an object (and stopping)
            self.goal_y = float(y)

            rospy.loginfo("Class: {}, X: {},  Y: {}".format(obj_class, x, y))

            camera = img_geo.StereoCameraModel()
            camera_info = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo, timeout=None)
          #  camera.fromCameraInfo(camera_info)

            xyz = Point()


            Z = self.depth

          #  u = camera.getDeltaU(self.goal_x, float(Z))
           # v = camera.getDeltaV(self.goal_y, float(Z))

            #uv_raw = (u,v)

            #uv = camera.rectifyPoint(uv_raw)

            u = x
            v = y
            disparity = camera.getDisparity(Z)
            xyz = camera.projectPixelTo3d((u, v), disparity)

            rospy.loginfo("Target: {}".format(xyz))

            self.object_locations.append(Point(float(xyz[0]),float(xyz[1]),0.0))
            self.is_target = True
            
            self.current_tgt_class = obj_class
            self.complete.append(self.current_tgt_class)

            # Break loop now we have target
            break

    def status_callback(self, msg):
        self.goal_status = msg.status.status

    def patrol(self):
        rospy.loginfo("Start patrol.")
        # Empty dictionaries {} evaluate to False
        while len(self.unvisited_nodes) != 0:
            if self.is_target == True:
                node = list(self.object_locations)[0]
                target = self.object_locations.pop(node)
            else:
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

        if self.is_target == True:
 
            goal.header.frame_id = "map"          
            
        else:
     
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
            rospy.loginfo(self.object_locations)
            self.goal_publisher.publish(goal)
            rospy.sleep(1)

        while self.goal_status != 3:
            pass

        rospy.loginfo("The robot has reached the destination.")
        return True

        # Create a client to send goal requests to the move_base server through
        # a SimpleActionClient
        # client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        #
        # while not client.wait_for_server(rospy.Duration.from_sec(10.0)):
        #     rospy.loginfo("Waiting for the move_base action server")
        #
        # goal = MoveBaseGoal()

        # important because "map" defines we are using map coordinates
        # and not coordinates relative to the robot
        # goal.target_pose.header.frame_id = "map"
        # goal.target_pose.header.stamp = rospy.Time.now()
        #
        # #set positions of the goal location
        # goal.target_pose.pose.position = coordinates
        # goal.target_pose.pose.orientation.x = 0.0
        # goal.target_pose.pose.orientation.y = 0.0
        # goal.target_pose.pose.orientation.z = 0.0
        # goal.target_pose.pose.orientation.w = 1.0
        #
        # rospy.loginfo("Sending goal location")
        # client.send_goal(goal)
        #
        # client.wait_for_result(rospy.Duration(60))
        #
        # if (client.get_state() ==  GoalStatus.SUCCEEDED):
        #     rospy.loginfo("You have reached the destination")
        #     return True
        # else:
        #     rospy.loginfo("The robot failed to reach the destination")
        #     return False


# When this file is run:
# if __name__ == '__main__':
#     # Initialize node.
#     rospy.init_node('tour')
#
#
#     for i, data in enumerate(waypoints):
#         node = data[0]
#         location = data[1]
#         move_to_target(location)


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