import math
from Workspace import Workspace
import random
import sklearn
from sklearn.neighbors import NearestNeighbors
import numpy as np
import shapely
from shapely.geometry import LineString
from shapely.geometry import Point
import matplotlib.pyplot as plt
from matplotlib import collections  as mc
import pylab as pl
from astar_search import astar_search
from SearchSolution import SearchSolution


class PRM:

    def __init__(self, workspace):
        self.workspace = workspace
        self.thetas = self.workspace.thetas
        self.roadmap = dict()
        self.edges = list()

    #in this method we want to randomly sample points in the graph
    def sampling(self):
        random_thetas = []
        for x in range (0, len(self.thetas)):
            random_int = random.uniform(0, 2*math.pi)
            random_thetas.append(random_int)
        random_theta = tuple(random_thetas)
        return random_theta

    #this method checks for collisions
    def collision(self, random_theta):
        end_points = self.workspace.calculate_end_points(random_theta)
        new_lines = self.workspace.calculate_lines(end_points)
        collision = self.workspace.collided(new_lines)
        if collision:
            return True
        return False

    #this method does the local planning, the variable manual toggles whether we use manual or scikit learn
    def local_planner(self, k, theta, manual):
        neighbors = None
        if manual:
            neighbors = self.k_nearest_neighbors_manual(k, theta)
        else:
            neighbors = self.k_nearest_neighbors(k, theta, 1)
        print ("neighbors in local planer:", neighbors)
        neighbors = neighbors[:k] #getting k neighbors
        print ("subset of neighbors:", neighbors)
        return neighbors

    #this method calculates the k nearest neighbors manually
    def k_nearest_neighbors_manual(self, k, theta):
        neighbors = dict()
        for key in self.roadmap.keys():
            dist_sum = self.angular_distance(theta, key)
            neighbors[dist_sum] = key
        key_list = sorted(neighbors)
        neighbors_list = []
        for key in key_list:
            if self.not_collided(neighbors[key], theta, 3):
                neighbors_list.append(neighbors[key])
        print ("neighbors:", neighbors_list)
        return neighbors_list
    
    #this method checks to see if two points can transition from one to the other without colliding
    def not_collided(self, key, theta, max_dist):
        euclidian = self.euclidian_dist(key, theta)
        if (euclidian < max_dist):
            return False
        return True

    #this method calculates the euclidian distance between two points in the theta space
    def euclidian_dist(self, theta1, theta2):
        euclidian = 0
        for count in range (0, len(theta1)):
            dif = theta1[count] - theta2[count]
            euclidian += dif*dif
        return math.sqrt(euclidian)

    #this method builds the road map
    def build_roadmap(self, max_count, manual):
        count = 0
        while count < max_count:
            random_theta = self.sampling()
            if not self.collision(random_theta):
                if len(self.roadmap.keys()) == 0:
                    self.roadmap[random_theta] = []
                else:
                    links = self.local_planner(2, random_theta, manual)
                    self.roadmap[random_theta] = links
                    self.add_edges(random_theta, links)
            count += 1
        for key, value in self.roadmap.items():
            if value == []:
                links = self.local_planner(2, key, manual)
                self.roadmap[key] = links

    #this method calculates the angular distance between two theta values
    def angular_distance(self, theta1, theta2):
        dist_sum = 0
        for elem in range (0, len(theta1)):
            dir1 = (theta1[elem] - theta2[elem])
            dir2 = (theta2[elem] - theta1[elem])
            if dir1 < 0:
                dir1 = dir1 + 2*math.pi
            elif dir2 < 0:
                dir2 = dir2 + 2*math.pi
            if dir1 < dir2:
                dist_sum += dir1
            else:
                dist_sum += dir2
        return dist_sum

    #this method is an angular distance heuristic
    def heuristic(self, theta, goal):
        return self.angular_distance(theta, goal)

    #this mehtod checks to see if you are at a goal node
    def goal_test(self, theta, goal):
        if theta == goal:
            return True
        return False

    #this mehthod does the entire probabilistic road map
    def prm(self, max_count, start, goal, manual):
        self.build_roadmap(max_count, manual) #building the roadmap
        print ("edges:", self.edges)
        #self.print_prm()
        #print("start in prm:", start)
        solution = self.query(start, goal, manual)
        print ("solution path:", solution.path)
        self.workspace.generate_final_graphic(solution.path)

    #this method does the querying 
    def query(self, start, goal, manual):
        start_point_on_prm = self.local_planner(1, start, manual)
        goal_point_on_prm = self.local_planner(1, goal, manual)
        #if both start and goal can connect to the road map...
        if start_point_on_prm != [] and goal_point_on_prm != []:
            #connect them
            self.connect_sg(start, goal, start_point_on_prm, goal_point_on_prm)
            #then do astar search
            solution = astar_search(self.roadmap, start, goal, self.heuristic, self.goal_test)
        return solution

    #this method connects the start and the goal to the roadmap and adds them to the edge list
    def connect_sg(self, start, goal, start_point_on_prm, goal_point_on_prm):
        start_tuple = tuple(start_point_on_prm[0])
        goal_tuple = tuple(goal_point_on_prm[0])
        start_tuple_list = [start_tuple]
        goal_tuple_list = [goal_tuple]
        self.roadmap[tuple(start)] = start_tuple_list
        self.roadmap[tuple(goal)] = goal_tuple_list
        self.roadmap[goal_tuple].append(goal)
        self.add_edges(start_tuple, start_tuple_list)
        self.add_edges(goal_tuple, goal_tuple_list)

    #this method adds edges to the edge lsit 
    def add_edges(self, theta, links):
        for point in links:
            if type(point) is list:
                point = point[0]
            new_edge = (theta, point)
            self.edges.append(new_edge)

    #this method prints the roadmap as it is being created (only if there are 2 angles beign used though)
    #it is used to help with conceptual visualization and debugging
    def print_prm(self):
        x = []
        y = []
        fig, ax = plt.subplots()
        for key in self.roadmap.keys():
            x.append(key[0])
            y.append(key[1])
        plt.plot(x,y, 'ro')
        lc = mc.LineCollection(self.edges, linewidths=2)
        ax.add_collection(lc)
        plt.axis([0, 2*math.pi, 0, 2*math.pi])
        plt.show()

    #this method formats the samples so they are in a list instead of a sequence
    def format_samples(self, samples):
        formatted_samples = []
        sequence = samples[0]
        for count in range (0, len(sequence)):
            formatted_samples.append(sequence[count])
        print ("sequence:", formatted_samples)
        return formatted_samples

    #this method gets the edges in the prm graph
    def get_edges(self):
        edges = []
        for key in self.roadmap.keys():
            value = self.roadmap[key]
            if value != []:
                edges.append(value)
            
        print ("edges:", edges)
        return edges

    #BONUSt
    #this returns the k nearest neighbors to a point using the scikit learn method
    def k_nearest_neighbors(self, k, theta, radius_dist):
        point_list = list(self.roadmap.keys())
        print ("point list:", point_list)
        samples = []
        if len(point_list) <= k:
            print ("not enough points to need k nearest neighbors")
            for point in point_list:
                if point != theta:
                    samples.append(point)
            print ("samples:", samples)
            return samples
        else:
            nbrs = NearestNeighbors(n_neighbors=k, radius=radius_dist) 
            nbrs.fit(point_list)
            print ("nearest neighbors:", nbrs)
             #this gives you the index in the samples list of what it is near to?
            samples = nbrs.kneighbors([theta], k, return_distance=False)
            samples = self.format_samples(samples)
            neighbors = []
            for sample in samples:
                neighbors.append(point_list[sample])
            print ("the k nearest neighbors are:", neighbors)
            return neighbors