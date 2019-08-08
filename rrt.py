# using shapely for collision detection
import numpy as np
import math

import random
import shapely
from shapely.geometry import Polygon, Point
from matplotlib import pyplot as plt
from matplotlib import collections  as mc
import matplotlib.patches as patches
from planar_trajectory import PlanarTrajectory
from planar_display import TrajectoryView, CarShapes
from planarsim import *

class rrt:

    #the initialization of this class 
    def __init__(self, start_point, max_distance, obstacles, dist_to_goal):
        self.start_point = start_point
        self.max_distance = max_distance
        self.figsize = (8, 4.5)
        self.obstacles = obstacles
        self.rrt_graph = self.start_configuration(start_point) #this is a dictionary with vertexes as keys and edges as a list
        self.dist_to_goal = dist_to_goal
        self.state_action = dict()
        self.point_list = [start_point]
    
    #this method sets up the start configuraiton
    def start_configuration(self, start_point):
        rrtg = dict()
        #this sets the root node of the rrt graph
        rrtg[None] = [start_point]
        return rrtg

    #this generates a random point with an x, y, and theta value
    def random_point(self):
        rand_x = random.uniform(-8, 8)
        rand_y = random.uniform(-4.5, 4.5)
        rand_theta = random.uniform(0, 2*math.pi)
        rand_point = (rand_x, rand_y, rand_theta)
        return rand_point

    def rrt_complete(self, num_vertices, goal):
        final_path = self.rrt_alg(num_vertices, goal)
        if final_path != None:
            self.finish_rrt(final_path)
        else:
            print ("Goal not found")


    #this method generates the rrt graph
    def rrt_alg(self, num_vertices, goal):
        print ("Goal:", goal)
        for k in range (0, num_vertices):
            near_point = None
            if k == 0:
                near_point = self.start_point
            else:
                #find a random point in the space
                rand_point = self.random_point()
                #find the nearest point on the graph already to this new point
                near_point = self.local_planner(1, rand_point)
                near_point = near_point[0]
            children = []
            #simulate the six actions in the control_rs list
            for key in range (0, 6):
                traj = PlanarTrajectory(controls_rs, near_point[0], near_point[1], near_point[2], [key], [1.01])
                x, y, theta = traj.config_at_t(1.0)
                next_point = (x, y, theta)
                #check if collision
                if not self.collided(next_point):
                    #make sure the edges can be added to the graph
                    if self.can_add_point(near_point, next_point):
                        self.point_list.append(next_point)
                        #then add points to children list
                        children.append(next_point)
                        #puts the action that got us to a given state into a dict for future lookup
                        self.state_action[next_point] = key 
                        #if the point is close enough to the goal then backchain
                        if self.distance(next_point, goal) < self.dist_to_goal:
                            self.visualize_rrt()
                            final_path = self.backchain(next_point, key, near_point, num_vertices)
                            return final_path
            #add children list to parent key
            self.rrt_graph[near_point] = children
        return
                        

    #this method checks to make sure we can add edge because the two points are close enough
    def can_add_point(self, point1, point2):
        dist_between_pts = self.distance(point1, point2)
        if dist_between_pts < self.max_distance:
            return True
        return False
        
    #this is the method that checks if the car has collided with any of the obstacles
    def collided(self, curr_point):
        for obstacle in self.obstacles:
            vertex_list = [(obstacle[0], obstacle[1]), (obstacle[0], obstacle[1] + obstacle[3]), 
                (obstacle[0] + obstacle[2], obstacle[1]), (obstacle[0] + obstacle[2], obstacle[1] + obstacle[3])]
            polygon = Polygon(vertex_list)
            x = curr_point[0]
            y = curr_point[1]
            point = Point(x, y)
            if polygon.contains(point):
                return True
        return False


    #this method does the local planning 
    def local_planner(self, k, pos):
        #neighbors = self.k_nearest_neighbors(k, theta)
        neighbors = self.k_nearest_neighbors_manual(pos)
        neighbors = neighbors[:k] #getting k neighbors
        return neighbors

    #this method calculates the k nearest neighbors manually
    def k_nearest_neighbors_manual(self, pos):
        neighbors = dict()
        for point in self.point_list:
            if point != None:
                dist_sum = self.distance(pos, point)
                neighbors[dist_sum] = point
        key_list = sorted(neighbors)
        neighbors_list = []
        for key in key_list:
            neighbors_list.append(neighbors[key])
        return neighbors_list

    #this method returns the distance between two points in the (x, y, theta) space
    def distance(self, pos, key):
        #k is the amount of the theta distance that should be taken into consideration
        #i.e. a scaled version of the theta distance
        k = 10/(2*math.pi)
        difx = abs(pos[0] - key[0])
        dify = abs(pos[1] - key[1])
        ang_dist = self.angular_distance(pos[2], key[2])
        dist = difx + dify + k*ang_dist
        return dist

    #this method calculates the angular distance between two theta values
    def angular_distance(self, theta1, theta2):
        dist_sum = 0
        dir1 = (theta1 - theta2)
        dir2 = (theta2 - theta1)
        if dir1 < 0:
            dir1 = dir1 + 2*math.pi
        elif dir2 < 0:
            dir2 = dir2 + 2*math.pi
        if dir1 < dir2:
            dist_sum += dir1
        else:
            dist_sum += dir2
        return dist_sum

    #this method backchains from the final point, the point close enough to the goal, to the start
    def backchain(self, final_point, index, near_point, num_vertices):
        path = []
        path.append(index)
        next_point = near_point
        while next_point != self.start_point:
            #we want to append the actions to the path list so that they can then be paseed
            #planar trajectory to be turned into an animation/image
            action_next = self.state_action[next_point] 
            path.append(action_next)
            values_list = list(self.rrt_graph.values())
            count = 0
            while count < len(values_list): 
                if next_point in values_list[count]:
                    next_point = list(self.rrt_graph.keys())[count]
                    count = len(values_list)
                count += 1
        path.reverse()
        print ("path:", path)
        return path

    #this method is called when we have reached a point close enough to the goal point
    #it completes the rrt
    def finish_rrt(self, final_path):
        duration_list = [2.0] * len(final_path)
        fig = plt.figure(figsize=(8, 4.5))
        fig.tight_layout()
        plt.axis([-10, 10, -8, 8])
        traj = PlanarTrajectory(controls_rs, self.start_point[0], self.start_point[1], self.start_point[2], final_path, duration_list)
        tview = TrajectoryView(traj)
        ax = plt.axes()
        finalt = traj.end_time - .01
        print ("length of list:", len(final_path))
        print ("end time:", finalt)
        tview.draw(ax, finalt)
        plt.show()

    def visualize_rrt(self):
        x = []
        y = []
        fig, ax = plt.subplots()
        for state in self.state_action.keys():
            x.append(state[0])
            y.append(state[1])
        plt.plot(x,y, 'ro')
        edges = []
        for key in self.rrt_graph.keys():
            if key != None:
                for value in self.rrt_graph[key]:
                    edge = [(key[0], key[1]), (value[0], value[1])]
                    edges.append(edge)
        lc = mc.LineCollection(edges, linewidths=2)
        ax.add_collection(lc)
        plt.axis([-10, 10, -8, 8])
        for obstacle in self.obstacles:
            rect = patches.Rectangle((obstacle[0],obstacle[1]),obstacle[2],obstacle[3],linewidth=1, edgecolor='r',facecolor='r')
            ax.add_patch(rect)
        plt.show()



