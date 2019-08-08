from SearchSolution import SearchSolution
from heapq import heappush, heappop

class AstarNode:
    # each search node except the root has a parent node
    # and all search nodes wrap a state object

    def __init__(self, roadmap, theta, heuristic_fn, parent=None, transition_cost=0):
        # you write this part
        self.roadmap = roadmap
        self.theta = theta
        self.parent = parent
        self.heuristic_value = heuristic_fn
        self.transition_cost = transition_cost

    def priority(self):
        priority = self.transition_cost + self.heuristic_value
        return (priority)

    #this method gets the successors for a given node
    def get_successors(self):
        num_thetas = len(self.theta)
        successors_list = self.roadmap[tuple(self.theta)]
        print ("successors list:", successors_list)
        all_successors = []
        if self.parent == None:
            for x in range (0, len(successors_list), num_thetas):
                if num_thetas == 2:
                    elem1 = successors_list[0][x]
                    elem2 = successors_list[0][x+1]
                    elem_list = [elem1, elem2]
                elif num_thetas == 3:
                    elem1 = successors_list[0][x]
                    elem2 = successors_list[0][x+1]
                    elem3 = successors_list[0][x+2]
                    elem_list = [elem1, elem2, elem3]
                elif num_thetas == 4:
                    elem1 = successors_list[0][x]
                    elem2 = successors_list[0][x+1]
                    elem3 = successors_list[0][x+2]
                    elem4 = successors_list[0][x+3]
                    elem_list = [elem1, elem2, elem3, elem4]
                all_successors.append(tuple(elem_list))
        else:
            for point in range (0, len(successors_list)):
                elem_list = []
                for elem in range (0, num_thetas):
                    elem_list.append(successors_list[point][elem])
                #print ("elem list:", elem_list)
                all_successors.append(tuple(elem_list))
        return all_successors

    # comparison operator,
    # needed for heappush and heappop to work with AstarNodes:
    def __lt__(self, other):
        return self.priority() < other.priority()

# take the current node, and follow its parents back
#  as far as possible. Grab the states from the nodes,
#  and reverse the resulting list of states.
def backchain(node, solution, final_visited_cost):
    solution = solution
    result = []
    solution.cost = final_visited_cost
    #print "final cost", solution.cost
    current = node
    while current:
        result.append(current.theta)
        current = current.parent
    result.reverse()
    solution.path = result
    return solution


def astar_search(roadmap, start, goal, heuristic_fn, goal_fn):
    # I'll get you started:
    print ("ASTAR STARTING", "\n")
    print ("roadmap:", roadmap)
    print ("start state:", start)
    print ("goal state:", goal)
    start_node = AstarNode(roadmap, tuple(start), heuristic_fn(start, goal))
    start_node.transition_cost = 0
    pqueue = []
    heappush(pqueue, start_node)

    solution = SearchSolution(start, goal, "Astar with heuristic " + heuristic_fn.__name__)

    visited = dict()
    visited[tuple(start_node.theta)] = start_node
    
    #while there is something left in the queue
    while pqueue:
        node = heappop(pqueue) #pop off the lowest value thing
        if (node.transition_cost == -1):
            continue
        else:
            current_theta = node.theta #get the current state from that node
            visited[tuple(current_theta)] = node #put the node into the dictionary with the state as the key
            if goal_fn(current_theta, goal): #if it is the goal
                print ("SOLUTION FOUND")
                return backchain(node, solution, node.transition_cost) #return the backchain solution
            successors_list = node.get_successors() #generate the sucessors list
            for child in successors_list: #for each child
                new_cost = node.transition_cost + 1
                if tuple(child) not in list(visited.keys()):
                    new_node = AstarNode(roadmap, child, heuristic_fn(child, goal), node, new_cost) #create a new node with the new cost and the child
                    heappush(pqueue, new_node) #put that node onto the heap
                    visited[child] = new_node #put it in the visited cost list
                elif (visited.get(child).transition_cost > new_cost): #if the cost of the node already in the list is bigger than the new cost
                    new_node = AstarNode(roadmap, child, heuristic_fn(child, goal), node, new_cost) #create the new node
                    old_node= visited[child] #get the old, worse node
                    old_node.transition_cost = -1 #make its transition cost -1 so it isn't pulled out of the heap
                    visited[child] = new_node #make the value associated with the state in the dictionary be the new, better node 
                    heappush(pqueue, new_node) #push new node onto heap
                new_cost = 0 #resetting the new cost value
            solution.nodes_visited += 1 #increase the number of nodes visited 
    return solution


        

        

