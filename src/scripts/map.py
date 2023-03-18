import numpy as np
import cv2 as cv
import heapq


class Map:
    def __init__(self, dimensions, resolution, detection_cfg):
        self.dimensions = dimensions
        self.resolution = resolution
        self.detection_cfg = detection_cfg
        self.garage = None
        self.gate = None
        self.obstacles = []
        self.robot = None

        x = self.conv_real_to_map(dimensions[0])
        y = self.conv_real_to_map(dimensions[1])
        self.world_map = np.zeros((x, y), dtype=np.uint8)

    # Functionalities
    def find_way(self, start, goal) -> np.ndarray:
        """
        Use A* algorithm to find the way from the robot to the garage on the map.
        :param start: The starting point.
        :param goal: The ending point.
        :return: The path from the robot to the garage. The path is a list of points.
        """
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
            for i, j in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = current_node[0] + i, current_node[1] + j

                # Check if the neighbor is within the maze bounds and is not an obstacle
                if 0 <= neighbor[0] < self.world_map.shape[0] and 0 <= neighbor[1] < self.world_map.shape[1] \
                        and self.world_map[neighbor] < 2:

                    # Compute the tentative cost for the neighbor
                    tentative_cost = cost[current_node] + 1

                    # Check if the neighbor has already been evaluated or if the tentative cost is lower
                    if neighbor in closed_set and tentative_cost >= cost.get(neighbor, float('inf')):
                        continue

                    # Add the neighbor to the open set and update the cost and parent dictionaries
                    if tentative_cost < cost.get(neighbor, float('inf')):
                        cost[neighbor] = tentative_cost
                        priority = tentative_cost + self.heuristic(goal, neighbor)
                        heapq.heappush(open_set, (priority, neighbor))
                        parent[neighbor] = current_node
        # If the goal cannot be reached
        return None

    def heuristic(self, a, b) -> float:
        """
        Calculate the heuristic distance between two points.
        :param a: The first point.
        :param b: The second point.
        :return: The heuristic distance between the two points.
        """
        heuristic = self.detection_cfg['map']['heuristic']
        if heuristic == "euclidean":
            return np.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
        elif heuristic == "manhattan":
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        elif heuristic == 'chebyshev':
            return max(abs(a[0] - b[0]), abs(a[1] - b[1]))
        elif heuristic == "octile":
            dx = abs(a[0] - b[0])
            dy = abs(a[1] - b[1])
            return dx + dy + (np.sqrt(2) - 2) * min(dx, dy)
        elif heuristic == "none":
            return 0
        else:
            raise ValueError("Unknown heuristic")

    def fill_world_map(self) -> None:
        """
        Fill the world map with the objects which were found by detector.
        :return: None
        """
        map_cfg = self.detection_cfg['map']

        # Garage
        if self.garage is not None:
            garage_id = map_cfg['id']['garage']

            # Convert real world parameters to map parameters
            x = self.conv_real_to_map(self.garage.get_world_coordinates()[0], add=True)
            y = self.conv_real_to_map(self.garage.get_world_coordinates()[1])
            length = self.conv_real_to_map(self.garage.get_length())
            width = self.conv_real_to_map(self.garage.get_width())

            # Draw the garage
            cv.rectangle(self.world_map, (x, y), (x+length, y+width), garage_id, -1)

        # Obstacles
        for obstacle in self.obstacles:
            obst_id = map_cfg['id']['obstacle']

            # Convert real world parameters to map parameters
            x = self.conv_real_to_map(obstacle.get_world_coordinates()[0], add=True)
            y = self.conv_real_to_map(obstacle.get_world_coordinates()[1])
            radius = self.conv_real_to_map(obstacle.get_radius())

            cv.circle(self.world_map, (x, y), radius, obst_id, -1)

        # Robot
        if self.robot is not None:
            robot_id = map_cfg['id']['robot']

            # Convert real world parameters to map parameters
            x = self.conv_real_to_map(self.robot.get_world_coordinates()[0], add=True)
            y = self.conv_real_to_map(self.robot.get_world_coordinates()[1])
            radius = self.conv_real_to_map(self.robot.get_radius())

            # Draw the robot
            cv.circle(self.world_map, (x, y), radius, robot_id, -1)

        # Gate
        if self.gate is not None:
            gate_id = map_cfg['id']['gate']

            # Convert real world radius to map radius
            radius = self.conv_real_to_map(self.gate.get_width()/2)
            for slope in self.gate.get_world_coordinates():
                # Convert real world parameters to map parameters
                x = self.conv_real_to_map(slope[0], add=True)
                y = self.conv_real_to_map(slope[1])

                # Draw the slope
                cv.circle(self.world_map, (x, y), radius, gate_id, -1)

    def conv_real_to_map(self, realc, add=False) -> int:
        """
        Convert realc to map.
        :param realc: The real dims.
        :param add: If true, add the map center to the realc.
        """
        mapc = int(realc / self.resolution)
        if add:
            mapc += self.world_map.shape[0] // 2
        return mapc

    # SETTERS
    def set_dimensions(self, dimensions):
        self.dimensions = dimensions

    def set_resolution(self, resolution):
        self.resolution = resolution

    def set_garage(self, garage):
        self.garage = garage

    def set_gate(self, gate):
        self.gate = gate

    def set_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def set_robot(self, robot):
        self.robot = robot

    def set_world_map(self, world_map):
        self.world_map = world_map

    # GETTERS
    def get_dimensions(self):
        return self.dimensions

    def get_resolution(self):
        return self.resolution

    def get_garage(self):
        return self.garage

    def get_gate(self):
        return self.gate

    def get_obstacles(self):
        return self.obstacles

    def get_robot(self):
        return self.robot

    def get_world_map(self):
        return self.world_map