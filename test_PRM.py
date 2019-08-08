from Workspace import Workspace
from sklearn.neighbors import NearestNeighbors
from PRM import PRM
import math


# #testing PRM simple 2 arm segments, with the manual k nearest neighbors

# start = (0, 0)
# goal = (3*math.pi/2, 3*math.pi/2)
# lengths = [2, 2]
# obstacles = [[0, 4, 1, 6]]
# test_arm = Workspace(start, lengths, obstacles)
# #test_arm.generate_graphic()

# test_prm = PRM(test_arm)
# #test_prm.print_prm()
# test_prm.prm(10, start, goal, True)
# #test_arm.generate_graphic()

# #the same test but using the scikit learn k nearest neighbors 

# start = (0, 0)
# goal = (3*math.pi/2, 3*math.pi/2)
# lengths = [2, 2]
# obstacles = [[0, 4, 1, 6]]
# test_arm = Workspace(start, lengths, obstacles)
# #test_arm.generate_graphic()

# test_prm = PRM(test_arm)
# #test_prm.print_prm()
# test_prm.prm(10, start, goal, False)
# #test_arm.generate_graphic()

# #3 arm segments

# start = (0, 0, 0)
# goal = (3*math.pi/2, 3*math.pi/2, 0)
# lengths = [2, 5, 2]
# obstacles = [[0, 4, 1, 6], [4, 4, 2, 2]]
# test_arm = Workspace(start, lengths, obstacles)
# #test_arm.generate_graphic()

# test_prm = PRM(test_arm)
# #test_prm.print_prm()
# test_prm.prm(40, start, goal, True)
# #test_arm.generate_graphic()

# #4 arm segments

start = (0, 0, 0, 0)
goal = (3*math.pi/2, 3*math.pi/2, 0, math.pi/2)
lengths = [2, 2, 2, 2]
obstacles = [[-2, -1, 1, 6], [4, 4, 2, 2]]
test_arm = Workspace(start, lengths, obstacles)
#test_arm.generate_graphic()

test_prm = PRM(test_arm)
#test_prm.print_prm()
test_prm.prm(40, start, goal, True)
#test_arm.generate_graphic()

