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
        self.goal = None

    # BEGIN: Path finding
    def find_way(self, start, goal, search_algorithm) -> np.ndarray:
        """
        Find the way from the robot to the garage on the map.
        :param start: The starting point.
        :param goal: The ending point.
        :param search_algorithm: The search algorithm to use.
        :return: The path from the robot to the garage. The path is a list of points.
        """
        if search_algorithm == "A_star":
            return self.a_star(start, goal)
        elif search_algorithm == "BFS":
            return self.bfs(start, goal)
        else:
            raise ValueError("Unknown search algorithm")

    def a_star(self, start, goal):
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
                if 0 <= neighbor[0] < self.world_map.shape[0] and 0 <= neighbor[1] < self.world_map.shape[1] and \
                        self.world_map[neighbor] < 2:

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

    # END: Path finding

    # BEGIN: Map filling
    def fill_world_map(self) -> None:
        """
        Fill the world map with the objects which were found by detector.
        :return: None
        """
        # Obstacles
        for obstacle in self.obstacles:
            self.draw_restricted_area(obstacle.get_world_coordinates(), obstacle.get_radius())
            self.fill_in_obstacle(obstacle)

        # Gate
        pillars = []
        if self.gate is not None:
            pillars = self.fill_in_gate()
            self.gate.calculate_orientation()

        # Garage (if the gate has been found we can predict position of the garage)
        self.fill_in_garage(pillars)

        # Robot
        if self.robot is not None:
            self.fill_in_robot()

    def fill_in_garage(self, pillars) -> None:
        """
        Fill the garage in the world map.
        :param pillars: The map coordinates of the gate's pillars.
        """
        garage_id = self.detection_cfg['map']['id']['garage']

        # If the gate has been found, we can predict the position of the garage
        if len(pillars) == 2:
            garage_width = self.conv_real_to_map(self.gate.get_garage_dimensions_lwh()[1])
            garage_length = self.conv_real_to_map(self.gate.get_garage_dimensions_lwh()[0])

            # Get the orientation of the gate
            orientation = self.gate.get_orientation()

            # Find the other two corner points of the garage
            if orientation < np.pi/2:
                # Get the reference pillar (the one which is on the left side of the gate)
                ref_pillar = pillars[0] if pillars[0][0] < pillars[1][0] else pillars[1]
                p1 = self.calculate_next_point(ref_pillar, np.pi/2 + orientation, garage_width)
                p2 = self.calculate_next_point(p1, orientation, garage_length)
            else:
                # Get the reference pillar (the one which is on the right side of the gate)
                ref_pillar = pillars[0] if pillars[0][0] > pillars[1][0] else pillars[1]
                p1 = self.calculate_next_point(ref_pillar, orientation-np.pi/2, garage_width)
                p2 = self.calculate_next_point(p1, orientation, garage_length)

            # Connect these points by lines
            other_pillar = pillars[0] if pillars[0] != ref_pillar else pillars[1]
            cv.line(self.world_map, ref_pillar, p1, garage_id, 2)
            cv.line(self.world_map, p1, p2, garage_id, 2)
            cv.line(self.world_map, p2, other_pillar, garage_id, 2)

            # Find a mirror point of the p1 point with respect to the reference pillar
            p3 = self.calculate_next_point(ref_pillar, orientation, garage_length)              # TODO: delete
            p4 = self.calculate_next_point(p3, orientation-np.pi/2, garage_width)               # TODO: delete

            # Set center of p4 and ref_pillar as the goal
            # self.goal = ((p4[0] + ref_pillar[0]) // 2, (p4[1] + ref_pillar[1]) // 2)          # TODO: delete

            # Calculate the center point between the two pillars
            center = ((ref_pillar[0] + other_pillar[0]) // 2, (ref_pillar[1] + other_pillar[1]) // 2)

            # Calculate perpendicular vector to the vector between the two pillars
            v = (other_pillar[0] - ref_pillar[0], other_pillar[1] - ref_pillar[1])
            v_perp = (-v[1], v[0])

            # Calculate the two points on the perpendicular vector which are on the line between the two pillars
            p5 = (center[0] + v_perp[0], center[1] + v_perp[1])
            p6 = (center[0] - v_perp[0], center[1] - v_perp[1])

            # Set the one with smaller y coordinate as the goal
            self.goal = p5 if p5[1] < p6[1] else p6

        # If only one or none of the pillars has been found, we cannot predict the position of the garage
        # We will fill in points of the garage itself (yellow area in RGB image)
        elif len(pillars) == 1 or len(pillars) == 0:
            # Get the coordinates of the garage
            if self.garage is not None:
                coords = self.garage.get_world_coordinates()
                xs = coords[0]
                ys = coords[1]
                for i in range(len(xs)):
                    x = self.conv_real_to_map(xs[i], add=True)
                    y = self.conv_real_to_map(ys[i])
                    cv.circle(self.world_map, (x, y), 1, garage_id, -1)

            # TODO: DELETE
            self.goal = (100, 100)
        else:
            raise ValueError("The gate has more than 2 pillars.")

    def fill_in_gate(self) -> list:
        """
        Fill the gate in the world map.
        :return: The map coordinates of the gate's pillars.
        """
        gate_id = self.detection_cfg['map']['id']['gate']

        # Convert real world radius to map radius
        radius = self.conv_real_to_map(self.gate.get_width())
        pillars = []
        for pillar in self.gate.get_world_coordinates():
            # Convert real world parameters to map parameters
            x = self.conv_real_to_map(pillar[0], add=True)
            y = self.conv_real_to_map(pillar[1])
            pillars.append((x, y))

            # Draw the pillar
            cv.circle(self.world_map, (x, y), radius, gate_id, -1)
        return pillars

    def fill_in_obstacle(self, obstacle: object) -> None:
        """
        Fill the obstacle in the world map.
        :param obstacle: The obstacle to be filled in.
        :return: None
        """
        obst_id = self.detection_cfg['map']['id']['obstacle']

        # Convert real world parameters to map parameters
        x = self.conv_real_to_map(obstacle.get_world_coordinates()[0], add=True)
        y = self.conv_real_to_map(obstacle.get_world_coordinates()[1])
        radius = self.conv_real_to_map(obstacle.get_radius())

        cv.circle(self.world_map, (x, y), radius, obst_id, -1)

    def fill_in_robot(self) -> None:
        """
        Fill the robot in the world map.
        :return: None
        """
        robot_id = self.detection_cfg['map']['id']['robot']

        # Convert real world parameters to map parameters
        x = self.conv_real_to_map(self.robot.get_world_coordinates()[0], add=True)
        y = self.conv_real_to_map(self.robot.get_world_coordinates()[1])
        radius = self.conv_real_to_map(self.robot.get_radius())

        # Draw the robot
        cv.circle(self.world_map, (x, y), radius, robot_id, -1)

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

    def draw_restricted_area(self, center, size) -> None:
        """
        Draw restricted area around objects on the map.
        :param center: Center of the object.
        :param size: Size of the object.
        """
        # Convert real world parameters to map parameters
        x = self.conv_real_to_map(center[0], add=True)
        y = self.conv_real_to_map(center[1])
        size = self.conv_real_to_map(size)

        # Add half of the robot radius to the radius plus safety margin
        size += self.conv_real_to_map(self.robot.get_radius()) \
                  + self.conv_real_to_map(self.detection_cfg['map']['safety_margin'])

        # Get id of the restricted area
        id = self.detection_cfg['map']['id']['restricted']

        # Draw the restricted area as a square
        # cv.rectangle(self.world_map, (x - size, y - size), (x + size, y + size), id, -1)
        self.draw_octagon((x, y), size, id)

    def draw_octagon(self, center, side_length, color) -> None:
        """
        Draw a octagon on the map.
        :param center: The center of the pentagon.
        :param side_length: The side length of the pentagon.
        :param color: The color of the pentagon.
        """

        # Calculate the coordinates of the five vertices of the pentagon
        vertices = []
        for i in range(8):
            x = int(center[0] + side_length * np.cos(2 * np.pi * i / 8 + np.pi / 8))
            y = int(center[1] + side_length * np.sin(2 * np.pi * i / 8 + np.pi / 8))
            vertices.append((x, y))

        # Draw the octagon
        cv.fillPoly(self.world_map, [np.array(vertices)], color)

    @staticmethod
    def calculate_next_point(point, angle, distance) -> tuple:
        """
        Calculate the next point based on the current point, the angle and the distance.
        :param point: The current point.
        :param angle: The angle.
        :param distance: The distance.
        :return: The next point.
        """
        x = point[0] + distance * np.cos(angle)
        y = point[1] + distance * np.sin(angle)
        return int(x), int(y)

    # END: Map filling

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

    def set_goal(self, goal):
        self.goal = goal

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

    def get_goal(self):
        return self.goal