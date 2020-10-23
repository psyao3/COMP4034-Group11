#!/usr/bin/env python

import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(trajectory):
    coordinates = np.array(trajectory)
    x_coordinates = coordinates[:,0]
    y_coordinates = coordinates[:,1]
    plt.plot(x_coordinates,y_coordinates, color='lightblue', linewidth=3)
    plt.grid()
    plt.show()