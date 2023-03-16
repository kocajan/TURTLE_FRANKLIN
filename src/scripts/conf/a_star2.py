import numpy as np
import heapq
import matplotlib.pyplot as plt

def astar(maze, start, goal):
    """
    Implementation of A* algorithm for finding the shortest path in a maze from start to goal.
    Assumes that the maze is a numpy array with values 0 (for empty cells), 1 (for the goal cell), and any positive integer (for obstacles).
    """
    # Define the heuristic function (Manhattan distance)
    def heuristic(a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    # Initialize the open and closed sets
    open_set = []
    closed_set = set()

    # Add the starting node to the open set
    heapq.heappush(open_set, (0, start))

    # Initialize the cost dictionary and parent dictionary
    cost = {start: 0}
    parent = {start: None}

    # Iterate until the goal is reached or the open set is empty
    while open_set:
        # Pop the node with the lowest cost from the open set
        current_cost, current_node = heapq.heappop(open_set)

        # Check if the goal has been reached
        if current_node == goal:
            path = []
            while current_node:
                path.append(current_node)
                current_node = parent[current_node]
            return path[::-1]

        # Add the current node to the closed set
        closed_set.add(current_node)

        # Check the neighbors of the current node
        for i, j in [(0, -1), (0, 1), (-1, 0), (1, 0)]:
            neighbor = current_node[0] + i, current_node[1] + j

            # Check if the neighbor is within the maze bounds and is not an obstacle
            if 0 <= neighbor[0] < maze.shape[0] and 0 <= neighbor[1] < maze.shape[1] and maze[neighbor] < 2:

                # Compute the tentative cost for the neighbor
                tentative_cost = cost[current_node] + 1

                # Check if the neighbor has already been evaluated or if the tentative cost is lower
                if neighbor in closed_set and tentative_cost >= cost.get(neighbor, float('inf')):
                    continue

                # Add the neighbor to the open set and update the cost and parent dictionaries
                if tentative_cost < cost.get(neighbor, float('inf')):
                    cost[neighbor] = tentative_cost
                    priority = tentative_cost + heuristic(goal, neighbor)
                    heapq.heappush(open_set, (priority, neighbor))
                    parent[neighbor] = current_node

    # If the goal cannot be reached
    return None


