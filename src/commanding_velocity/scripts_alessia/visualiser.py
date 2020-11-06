#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np
import math

def plot_trajectory(trajectory):
    coordinates = np.array(trajectory)
    x_coordinates = coordinates[:,0]
    y_coordinates = coordinates[:,1]
    plt.plot(x_coordinates,y_coordinates, color='lightblue', linewidth=3)
    plt.grid()
    max_xy = math.floor(max(max(x_coordinates), max(y_coordinates))) 
    padding = max_xy/4
    plt.xlim(-padding, max(x_coordinates) + padding)
    plt.ylim(-padding, max(y_coordinates) + padding)
    plt.show()