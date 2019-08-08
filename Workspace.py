import math
import matplotlib.pyplot as plt
import numpy as np
import pylab as pl
from matplotlib import collections  as mc
import matplotlib.patches as patches
import shapely
from shapely.geometry import Polygon
from shapely.geometry import LineString

class Workspace:

    #this method initializes the arm simulation
    def __init__(self, thetas, lengths, obstacles):
        self.thetas = thetas
        self.lengths = lengths
        self.coords = self.calculate_end_points(self.thetas)
        print ("End coordinates:", self.coords)
        self.lines = self.calculate_lines(self.coords)
        print ("Line points:", self.lines)
        self.colors = np.array([(0, 1, 0, 1), (0, 0, 1, 1), (1, 0, 0, 1), (0, 1, 0, 1)])
        self.obstacles = obstacles

    #this method calculates the end points and puts them into a list of tuples
    #it does this for as many joints (or thetas) as there are in the current problem
    def calculate_end_points(self, thetas):
        coords = []
        elem = 0
        while elem < len(thetas):
            if elem == 0:
                x = self.lengths[elem] * math.cos(thetas[elem])
                y = self.lengths[elem] * math.sin(thetas[elem]) 
            else:
                x = coords[elem - 1][0]
                y = coords[elem - 1][1]
                theta_elem = 0
                theta_sum = 0
                while theta_elem < elem:
                    theta_sum += thetas[theta_elem]
                    theta_elem += 1
                x += self.lengths[elem] * math.cos(thetas[elem] + theta_sum)
                y += self.lengths[elem] * math.sin(thetas[elem] + theta_sum)  
            xy_tuple = (x , y)
            coords.append(xy_tuple)
            elem += 1
        return coords

    #this method calculates the lines to then be drawn by the generate graphcis method
    def calculate_lines(self, coords):
        lines = []
        elem = 0
        while elem < len(coords):
            line = []
            if elem == 0:
                start = (0, 0)
                line.append(start)
                line.append(coords[elem])
            else:
                line.append(coords[elem - 1])
                line.append(coords[elem])
            lines.append(line)
            elem += 1
        return lines

    #this is the method that checks if the robot arm has collided with any of the obstacles
    def collided(self, lines):
        for obstacle in self.obstacles:
            vertex_list = [(obstacle[0], obstacle[1]), (obstacle[0], obstacle[1] + obstacle[3]), 
                (obstacle[0] + obstacle[2], obstacle[1]), (obstacle[0] + obstacle[2], obstacle[1] + obstacle[3])]
            polygon = shapely.geometry.Polygon(vertex_list)
            for line in lines:
                path = shapely.geometry.LineString(line)
                if path.intersects(polygon):
                    print ("COLLIDED")
                    return True
        print ("DID NOT COLLIDE")
        return False
        
    #this method draws the robot arm and obstacles at the start state
    def generate_graphic(self):
        lines = self.lines
        num_lines = len(lines)
        print ("number of lines:", num_lines)
        c = self.colors[:num_lines]
        lc = mc.LineCollection(lines, color = c, linewidths=2)
        fig, ax = plt.subplots()
        ax.add_collection(lc)
        ax.margins(0.1)
        plt.axis([-10, 10, -10, 10])
        for obstacle in self.obstacles:
            rect = patches.Rectangle((obstacle[0],obstacle[1]),obstacle[2],obstacle[3],linewidth=1, edgecolor='r',facecolor='r')
            ax.add_patch(rect)
        if self.collided(lines):
            rect = patches.Rectangle((-10, -10), 20, 20, linewidth=1, edgecolor='r', facecolor='r')
            ax.add_patch(rect)
        plt.show()

    #this method generates the final graphic
    def generate_final_graphic(self, path):
        all_lines = []
        num_thetas = len(path[0])
        colors = self.colors[:num_thetas]
        num_steps = 0
        for step in path:
            coords = self.calculate_end_points(step)
            print ("coords in step:", coords)
            lines = self.calculate_lines(coords)
            print ("lines in step:", lines)
            all_lines.append(lines)
            #all_coords.append(coords)
            num_steps += 1
        #print ("all coords:", all_coords)
        print ("all lines:", all_lines)
        fig, ax = plt.subplots()
        count = 1
        for step in all_lines:
            lc = mc.LineCollection(step, color = count * colors, linewidths=2)
            ax.add_collection(lc)
            count *= .8
        ax.margins(0.1)
        plt.axis([-10, 10, -10, 10])
        for obstacle in self.obstacles:
            rect = patches.Rectangle((obstacle[0],obstacle[1]),obstacle[2],obstacle[3],linewidth=1, edgecolor='r',facecolor='r')
            ax.add_patch(rect)
        plt.show()



