#!/usr/bin/env python

import numpy as np
import cv2


def generate_mask(hsv):
    # Colour slicing to identify green objects
    # Colour of green object is [60, 159, 82]
    lower_green = np.array([40, 139, 62])
    upper_green = np.array([80, 179, 102])
    # Range of green values to account for shade differences
    mask = cv2.inRange(hsv, lower_green, upper_green)
    return mask


def convert_to_hsv(msg, bridge):
    # Convert the image message to openCV type, scale size.
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    (h, w) = image.shape[:2]
    scale_size = 4
    image_resized = cv2.resize(image, (w / scale_size, h / scale_size))
    # Convert to the HSV colour space.
    hsv = cv2.cvtColor(image_resized, cv2.COLOR_BGR2HSV)
    return hsv


def find_closest_centroid(centroids, w):
    # Keep only the label that belongs to the object
    # whose centroid is closest to the center of the
    # robots vision, i.e, beacon towards current target
    # if there are multiple objects.
    # Matrix of x,y centroids, start at 1 to ignore background
    x_centroids = centroids[1:,0]
    min_index = np.argmin(abs(x_centroids - w/2))
    best_label = min_index + 1

    return best_label


def show_image(mask, masked_image):
    # Show current image
    cv2.imshow("Mask", mask)
    cv2.imshow("Masked image", masked_image)
    cv2.waitKey(3)