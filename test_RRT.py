from planar_trajectory import *
from planarsim import *
from rrt import rrt

#in this file there are different simple tests. You may have to run each one a couple of times
#because the probabilistic, random nature means that goal location won't always be found

#very simple initial test
start = (0, 0, 0)
goal = [3, 3, 0]
obstacles = [[4, 4, 1, 6]]

test_rrt = rrt(start, 3, obstacles, 2)
test_rrt.rrt_complete(20, goal)

# #middle difficulty test
# #this is more challenging because the necessary distance between point and goal is smaller
# #in addition there are more obstacles
# start = (0, 0, 0)
# goal = [-3, -3, 0]
# obstacles = [[-6, -6, 7, 1], [4, 4, 2, 2]]

# test_rrt = rrt(start, 3, obstacles, 1)
# test_rrt.rrt_complete(100, goal)

# #much more complex test
# #the start and goal are much further apart and there are more obstacles in the way
# start = (-5, -5, 0)
# goal = [5, 5, 0]
# obstacles = [[-6, -6, 7, 1], [-6, 6, 7, 1]]

# test_rrt = rrt(start, 3, obstacles, 1)
# test_rrt.rrt_complete(1000, goal)