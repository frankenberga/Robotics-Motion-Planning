import math
import matplotlib.pyplot as plt
import numpy as np
import pylab as pl
from matplotlib import collections  as mc

class Arm_sim:

    #this method initializes the arm simulation
    def __init__(self, thetas, lengths):
        self.thetas = thetas
        self.lengths = lengths
        self.coords = self.calculate_end_points()
        print ("End coordinates:", self.coords)
        self.lines = self.calculate_lines()
        print ("Line points:", self.lines)
        self.colors = np.array([(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1), (0, 1, 1, 0)])
        self.obstacles

    #this method calculates the end points and puts them into a list of tuples
    #it does this for as many joints (or thetas) as there are in the current problem
    def calculate_end_points(self):
        coords = []
        elem = 0
        while elem < len(self.thetas):
            print ("element", elem)
            if elem == 0:
                x = self.lengths[elem] * math.cos(self.thetas[elem])
                y = self.lengths[elem] * math.sin(self.thetas[elem]) 
            else:
                x = coords[elem - 1][0]
                y = coords[elem - 1][1]
                theta_elem = 0
                theta_sum = 0
                while theta_elem < elem:
                    theta_sum += self.thetas[theta_elem]
                    theta_elem += 1
                x += self.lengths[elem] * math.cos(self.thetas[elem] + theta_sum)
                y += self.lengths[elem] * math.sin(self.thetas[elem] + theta_sum)  
            xy_tuple = (x , y)
            coords.append(xy_tuple)
            elem += 1
        return coords

    #this method calculates the lines to then be drawn by the generate graphcis method
    def calculate_lines(self):
        lines = []
        elem = 0
        while elem < len(self.coords):
            line = []
            if elem == 0:
                start = (0, 0)
                line.append(start)
                line.append(self.coords[elem])
            else:
                line.append(self.coords[elem - 1])
                line.append(self.coords[elem])
            lines.append(line)
            elem += 1
        return lines

    #this method draws the robot arm 
    def generate_graphic(self):
        lines = self.lines
        num_lines = len(lines)
        c = self.colors[:num_lines]
        lc = mc.LineCollection(lines, color = c, linewidths=2)
        fig, ax = plt.subplots()
        ax.add_collection(lc)
        ax.margins(0.1)
        plt.axis([-10, 10, -10, 10])
        plt.show()

