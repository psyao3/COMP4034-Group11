#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Pose2D
import cv2, cv_bridge
import numpy as np

class Follower:
   def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
      self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


   def image_callback(self, msg):
      # Convert the image message to openCV type, scale size.
      image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
      (h, w) = image.shape[:2]
      image_resized = cv2.resize(image, (w/4,h/4))

      # Convert to the HSV colour space.
      hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
      
      # Colour slicing to identify green objects
      # Colour of green object is [60, 159, 82]
      lower_green = np.array([50, 140, 75])
      upper_green = np.array([70, 170, 100])

      mask = cv2.inRange(hsv, lower_green, upper_green)

      # If there are multiple green objects/targets, only follow one.
      # Remove the one further away from the image.
      num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
      
      if num_labels > 1:
         # Iterate over labels, keep largest one.
         best_area = 0
         best_label = 0
         for i in range(1, num_labels):
            area = stats[i, cv2.CC_STAT_AREA] 
            if area >= best_area:
               best_area = area
               best_label = i
         # Keep only best_label, remove others from image.
         mask = np.where(labels==best_label, np.uint8(255), np.uint8(0))
         obj_centroid = centroids[best_label, 0]
      else:
         # If there is only one label, then there is only background;
         # No object to beacon towards, so exit early. Need other behaviour
         # to move until an object comes into view.
         return

      # Now the mask only contains the largest target.
      # Can use the centroids already provided from connectedComponents
            
      
      target = cv2.bitwise_and(hsv, hsv, mask=mask)

      # Show current image
      cv2.imshow("Mask", mask)
      cv2.imshow("Masked image", target)
      cv2.waitKey(3)

      # Need Image width to get centroid of turtlebots view.
      _,w,_ = target.shape
      twist_msg =  Twist()

      # Implement a proportional controller to beacon towards it
      err = obj_centroid - w/2
      twist_msg.linear.x = 0.2
      twist_msg.angular.z = -float(err)/100
      self.pub.publish(twist_msg)
      


rospy.init_node('follower')
follower = Follower()
rospy.spin()