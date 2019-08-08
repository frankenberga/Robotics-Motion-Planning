class SearchSolution:
    def __init__(self, start, goal, search_method):
        self.start = start
        self.goal = goal
        self.search_method = search_method
        self.path = []
        self.nodes_visited = 0
        self.cost = 0

    def __str__(self):
        string = "----\n"
        string += "{:s}\n"
        string += "attempted with search method {:s}\n"

        if len(self.path) > 0:

            string += "number of nodes visited: {:d}\n"
            string += "solution length: {:d}\n"
            string += "cost: {:d}\n"
            string += "path: {:s}\n"

            string = string.format("start:", self.start, "goal:", self.goal, self.search_method,
                self.nodes_visited, len(self.path), self.cost, str(self.path))
        else:
            string += "no solution found after visiting {:d} nodes\n"
            string = string.format("start:", self.start, "goal:", self.goal, self.search_method, self.nodes_visited)

        return string
