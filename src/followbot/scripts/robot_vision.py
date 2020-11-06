#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
import numpy as np

class Follower:
   def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)


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
      target = cv2.bitwise_and(hsv, hsv, mask=mask)

      # Show current image
      cv2.imshow("Mask", mask)
      cv2.imshow("Masked image", target)
      cv2.waitKey(3)

      # Implement a proportional controller to beacon towards it
      


rospy.init_node('follower')
follower = Follower()
rospy.spin()