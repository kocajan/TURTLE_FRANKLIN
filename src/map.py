import numpy as np
import cv2 as cv
import heapq
import queue


# TODO: ------------------------ DELETE THIS ------------------------------
def generate_rectangle_points(rect_length, rect_width):
    """
    Generate 100 points on the sides of a rectangle with given length and width.
    """
    # Define number of points to generate
    num_points = 1000

    # Generate points on the sides of the rectangle
    X = np.zeros((2, num_points))
    X[0, :num_points//4] = np.linspace(0, rect_length, num_points//4)
    X[0, num_points//4:num_points//2] = rect_length
    X[0, num_points//2:3*num_points//4] = np.linspace(rect_length, 0, num_points//4)
    X[0, 3*num_points//4:] = 0
    X[1, :num_points//4] = 0
    X[1, num_points//4:num_points//2] = np.linspace(0, rect_width//2, num_points//4)
    # X[1, num_points//2:3*num_points//4] = rect_width
    # X[1, 3*num_points//4:] = np.linspace(rect_width, 0, num_points//4)

    # Add noise to the points
    X += np.random.normal(0, 0.5, X.shape)

    # Randomly shift and rotate the points (but keep it a rectangle) Be careful about the dimensions
    theta = np.random.uniform(-np.pi/2, np.pi/2)
    shift = np.random.uniform(5, 10, 2)
    X = (np.dot(np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]]), X).T + shift).T

    return X

# TODO: -------------------------------------------------------------------------------------


class Map:
    def __init__(self, dimensions, resolution, detection_cfg):
        self.dimensions = dimensions
        self.resolution = resolution
        self.detection_cfg = detection_cfg
        self.garage = None
        self.gate = None
        self.obstacles = []
        self.robot = None

        x = int(self.conv_real_to_map(dimensions[0]))
        y = int(self.conv_real_to_map(dimensions[1]))
        self.world_map = np.zeros((x, y), dtype=np.uint8)
        self.goal_calculated = None
        self.goal = None
        self.goal_type = None

    # BEGIN: Path searching
    def find_way(self, start, goal, search_algorithm) -> list:
        """
        Find the way from the robot to the garage on the map.
        :param start: The starting point.
        :param goal: The ending point.
        :param search_algorithm: The search algorithm to use.
        :return: The path from the robot to the garage. The path is a list of points.
        """
        # Convert (x, y) coordinates to (y, x) coordinates
        start = (start[1], start[0])
        goal = (goal[1], goal[0])

        # Choose the search algorithm
        if search_algorithm == "A_star":
            return self.a_star(start, goal)
        elif search_algorithm == "BFS":
            return self.bfs(start, goal)
        else:
            raise ValueError("Unknown search algorithm")

    def a_star(self, start, goal) -> list:
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
                    tmp_node = (current_node[1], current_node[0])
                    path.append(tmp_node)
                    current_node = parent[current_node]
                return path[::-1]

            # Add the current node to the closed set
            closed_set.add(current_node)

            # Check the neighbors of the current node
            for i, j in [(0, -1), (0, 1), (-1, 0), (1, 0), (-1, -1), (-1, 1), (1, -1), (1, 1)]:
                neighbor = current_node[0] + j, current_node[1] + i

                # Check if the neighbor is within the maze bounds and is not an obstacle
                if 0 <= neighbor[0] < self.world_map.shape[0] and 0 <= neighbor[1] < self.world_map.shape[1] and \
                        self.world_map[neighbor] < 4:

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
        return []

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

    def bfs(self, start, goal) -> list:
        """
        Use BFS algorithm to find the way from the robot to the garage on the map.
        :param start: The starting point.
        :param goal: The ending point.
        :return: The path from the robot to the garage. The path is a list of points.
        """
        q = queue.Queue()
        q.put(start)
        path = []

        while not q.empty():
            to_expand = q.get()
            for node in self.expand(to_expand):
                if node == goal:
                    return path
                path.append(node)
                q.put(node)

    def expand(self, node_to_expand) -> list:
        """
        Expand the node.
        :param node_to_expand: The node to expand.
        :return: The list of the neighbours of the node.
        """
        result = []
        neighbours = [(0, 1), (1, 1), (-1, 1), (1, 0), (-1, 0)]
        memory = {}

        memory[str(node_to_expand)] = None
        for tuple in neighbours:
            tmp = (node_to_expand[0] + tuple[0], node_to_expand[1] + tuple[1])
            if 0 <= tmp[0] < self.world_map.shape[0] and 0 <= tmp[1] < self.world_map.shape[1] and self.world_map[tmp] < 2 and tmp not in memory.keys():
                result.append(tmp)
        return result

    # END: Path searching

    # BEGIN: Map filling
    def fill_world_map(self) -> None:
        """
        Fill the world map with the objects which were found by detector.
        :return: None
        """
        # Gate
        pillars = []
        if self.gate is not None:
            pillars = self.fill_in_gate()
            self.gate.calculate_orientation()

        # Garage (if the gate has been found we can predict position of the garage)
        self.fill_in_garage(pillars)

        # Obstacles
        for obstacle in self.obstacles:
            self.draw_restricted_area(obstacle.get_world_coordinates(), obstacle.get_radius(), angle=np.pi/4)
            self.fill_in_obstacle(obstacle)

        # Robot
        if self.robot is not None:
            self.fill_in_robot()

        # Draw vision cone
        #self.draw_vision_cone()

        # Handle the goal
        self.calculate_goal(pillars=pillars)
        self.correct_goal()
        self.fill_in_goal()

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

            # Connect these points with lines
            other_pillar = pillars[0] if pillars[0] != ref_pillar else pillars[1]
            cv.line(self.world_map, ref_pillar, p1, garage_id, 2)
            cv.line(self.world_map, p1, p2, garage_id, 2)
            cv.line(self.world_map, p2, other_pillar, garage_id, 2)

        # If only one or none of the pillars has been found, we cannot predict the position of the garage
        # We will fill in points of the garage itself (yellow area in RGB image)
        elif len(pillars) == 1 or len(pillars) == 0:
            # Get the coordinates of the garage
            if self.garage is not None:
                coords = self.garage.get_world_coordinates()
                xs = self.conv_real_to_map(coords[0], add=True)
                ys = self.conv_real_to_map(coords[1])
                for x, y in zip(xs, ys):
                    cv.circle(self.world_map, (x, y), 1, garage_id, -1)
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
            if x is not None and y is not None and radius is not None:
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

        # Draw the obstacle
        if x is not None and y is not None and radius is not None:
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
        if x is not None and y is not None and radius is not None:
            cv.circle(self.world_map, (x, y), radius, robot_id, -1)

    def fill_in_goal(self) -> None:
        """
        Fill the goal and the calculated goal in the world map.
        """
        goal_id = self.detection_cfg['map']['id']['goal']
        calculated_goal_id = self.detection_cfg['map']['id']['goal_calculated']

        # Draw the goals
        if self.goal is not None:
            cv.circle(self.world_map, self.goal, 5, goal_id, -1)
        if self.goal_calculated is not None:
            cv.circle(self.world_map, self.goal_calculated, 5, calculated_goal_id, -1)

    def fit_and_fill_garage_rectangle(self) -> tuple:
        """
        Fit a rectangle (garage size) to the garage points and fill it in the world map.
        :return: points of the fitted rectangle and the lengths of the lines used to fit the rectangle
        """
        # Get dimensions of the garage
        garage_width = self.conv_real_to_map(self.garage.get_width())
        garage_length = self.conv_real_to_map(self.garage.get_length())

        # Get the detected points of the garage
        points = self.garage.get_world_coordinates()

        # Convert real world parameters to map parameters
        xs = self.conv_real_to_map(points[0], add=True)
        ys = self.conv_real_to_map(points[1])

        # Fit a rectangle of the garage size to the detected points
        if xs is not None and ys is not None:
            # The rectangle will be fitted to the points by fitting lines to the points
            # and finding the intersection points of these lines
            p1, p2, p3, p4, line_length1, line_length2 = self.fit_rectangle(xs, ys, garage_length, garage_width)

            # Make the points integers
            p1 = (int(p1[0]), int(p1[1]))
            p2 = (int(p2[0]), int(p2[1]))
            p3 = (int(p3[0]), int(p3[1]))
            p4 = (int(p4[0]), int(p4[1]))

            # Fill restricted area around the garage
            center = (int((p1[0] + p3[0]) / 2), int((p1[1] + p3[1]) / 2))

            # Get distance between the center and the corners of the garage
            size = garage_width/2 if garage_width > garage_length else garage_length/2

            # Calculate angle of the garage (make it from 0 to pi/2)
            angle = np.arctan2(p1[1] - p2[1], p1[0] - p2[0])
            if angle > np.pi / 2:
                angle -= np.pi / 2

            # Draw the restricted area
            self.draw_restricted_area(center, size, convert=False, angle=angle)

            # Fill the rectangle in the world map
            color = self.detection_cfg['map']['id']['garage']
            cv.line(self.world_map, p1, p2, color, 2)
            cv.line(self.world_map, p2, p3, color, 2)
            cv.line(self.world_map, p3, p4, color, 2)
            cv.line(self.world_map, p4, p1, color, 2)

            return p1, p2, p3, p4, line_length1, line_length2

    def fit_rectangle(self, xs, ys, garage_width, garage_length) -> tuple:
        """
        Fit a rectangle to the given points.
        :param xs: The x map coordinates of the points.
        :param ys: The y map coordinates of the points.
        :param garage_width: The width of the garage. (map size) TODO: swap width and length
        :param garage_length: The length of the garage. (map size)
        :return: The rectangle fitted to the points.
        """
        lines = []
        # The robot can see only 2 sides of the garage at a time
        for i in range(3):
            # Fit a line to the points
            xs, ys, line, inliers = self.fit_line(xs, ys)
            if line is not None:
                lines.append([line, inliers])

        # Robot can see only 1 side of the garage
        if len(lines) == 1:
            # Get the line paramethers (a, b : y = ax + b)
            a, b = lines[0][0]

            # Get the leftmost and rightmost points of the line (inliers)
            inliers = lines[0][1]
            leftmost = inliers[np.argmin(inliers[:, 0])]
            rightmost = inliers[np.argmax(inliers[:, 0])]

            # Project these points to the line
            leftmost[1] = a*leftmost[0] + b
            rightmost[1] = a*rightmost[0] + b

            # Choose reference point (probably the point that is closer to the robot)
            # --------------------------------------
            # Get robot coordinates
            robot_x = self.conv_real_to_map(self.robot.get_world_coordinates()[0], add=True)
            robot_y = self.conv_real_to_map(self.robot.get_world_coordinates()[1])
            robot_point = np.array([robot_x, robot_y])

            # Get the distance of the points to the robot
            left_dist = np.linalg.norm(leftmost - robot_point)
            right_dist = np.linalg.norm(rightmost - robot_point)

            # Choose the closer point as the reference point
            ref_point = leftmost if left_dist < right_dist else rightmost
            other_point = rightmost if left_dist < right_dist else leftmost
            # --------------------------------------

            # Get the angle of the line from the two points
            angle = np.arctan2(other_point[1] - ref_point[1], other_point[0] - ref_point[0])

            # Get length of the line
            length = np.linalg.norm(leftmost - rightmost)

            # Calculate the difference between the garage dimensions and the line length
            length_diff = np.abs(length - garage_length)
            width_diff = np.abs(length - garage_width)

            # Decide which side of the garage the robot sees
            visible_side_length = garage_length if length_diff < width_diff else garage_width
            non_visible_side_length = garage_width if length_diff < width_diff else garage_length

            # Find the rectangle points
            if angle < np.pi/2:
                angle += np.pi
                visible_side_length = -visible_side_length 
                
            p1 = ref_point
            p2 = self.calculate_next_point(p1, angle, visible_side_length)
            p3 = self.calculate_next_point(p2, angle - np.pi/2, non_visible_side_length)
            p4 = self.calculate_next_point(p3, angle - np.pi, visible_side_length)

            # Rename the sides
            first_line_length = visible_side_length
            second_line_length = non_visible_side_length

        # Robot can see 2 sides of the garage
        else:
            # Get the lines
            print(lines)
            line1 = lines[0][0]
            line2 = lines[1][0]
            

            # Get the intersection point of the lines
            inter_x = (line2[1] - line1[1]) / (line1[0] - line2[0])
            inter_y = line1[0] * inter_x + line1[1]
            inter_point = np.array([inter_x, inter_y])

            # Find points that are farthest from the intersection point on both lines
            # --------------------------------------
            # Get the inliers
            inliers1 = lines[0][1]
            inliers2 = lines[1][1]

            # Get the distances of the inliers to the intersection point
            dists1 = np.linalg.norm(inliers1 - inter_point, axis=1)
            dists2 = np.linalg.norm(inliers2 - inter_point, axis=1)

            # Get the farthest points
            farthest1 = inliers1[np.argmax(dists1)]
            farthest2 = inliers2[np.argmax(dists2)]

            # Project these points to the lines
            farthest1[1] = line1[0] * farthest1[0] + line1[1]
            farthest2[1] = line2[0] * farthest2[0] + line2[1]
            # --------------------------------------

            # Get lengths of the lines
            length1 = np.linalg.norm(inter_point - farthest1)
            length2 = np.linalg.norm(inter_point - farthest2)

            # Get angles of the lines
            angle1 = np.arctan2(farthest1[1] - inter_point[1], farthest1[0] - inter_point[0])
            angle2 = np.arctan2(farthest2[1] - inter_point[1], farthest2[0] - inter_point[0])

            # Calculate the difference between the garage dimensions and the line lengths
            length_diff1 = np.abs(length1 - garage_length)
            length_diff2 = np.abs(length2 - garage_length)
            width_diff1 = np.abs(length1 - garage_width)
            width_diff2 = np.abs(length2 - garage_width)

            # TODO: ------------------------------- DELETE THIS -------------------------------
            print("garage_length, garage_width")
            print(garage_length, garage_width)
            print("length1, length2")
            print(length1, length2)
            print("length_diff1, length_diff2, width_diff1, width_diff2, angle1, angle2")
            print(length_diff1, length_diff2, width_diff1, width_diff2, angle1, angle2)
            # TODO: ----------------------------------------------------------------------------

            # Decide which sides of the garage the robot sees
            # --------------------------------------
            # First case: the first line is the width of the garage and the second line is the length
            if length1 > garage_length and length_diff2 < width_diff2:
                if angle1 < angle2:
                    first_line_length = garage_length
                    second_line_length = garage_width
                    angle = angle2
                else:
                    first_line_length = garage_width
                    second_line_length = garage_length
                    angle = angle1
                print("here1")
            # Second case: the first line is the length of the garage and the second line is the width
            elif length2 > garage_length and length_diff1 < width_diff1:
                if angle2 < angle1:
                    first_line_length = garage_length
                    second_line_length = garage_width
                    angle = angle1
                else:
                    first_line_length = garage_width
                    second_line_length = garage_length
                    angle = angle2
                print("here2")
            # Third case: Something is wrong (the smallest difference will decide)
            else:
                print("here3")
                diffs = np.array([length_diff1, length_diff2, width_diff1, width_diff2])
                min_diff = np.argmin(diffs)
                # If the smallest difference is 0 or 2, then the first line is the length and the second is the width
                if min_diff == 0 or min_diff == 2:
                    print("here3.1")
                    if angle1 < angle2:
                        first_line_length = garage_length
                        second_line_length = garage_width
                        angle = angle2
                    else:
                        first_line_length = garage_width
                        second_line_length = garage_length
                        angle = angle1
                # If the smallest difference is 1 or 3, then the first line is the width and the second is the length
                else:
                    print("here3.2")
                    if angle2 < angle1:
                        first_line_length = garage_length
                        second_line_length = garage_width
                        angle = angle1
                    else:
                        first_line_length = garage_width
                        second_line_length = garage_length
                        angle = angle2
            # --------------------------------------

            # Find the rectangle points
            p1 = inter_point
            p2 = self.calculate_next_point(p1, angle, first_line_length)
            p3 = self.calculate_next_point(p2, angle - np.pi/2, second_line_length)
            p4 = self.calculate_next_point(p3, angle - np.pi, first_line_length)

        return p1, p2, p3, p4, first_line_length, second_line_length

    def fit_line(self, xs, ys) -> (np.ndarray, np.ndarray, (float, float), np.ndarray):
        """
        Fit a line to the given points using RANSAC.
        :param xs: The x coordinates of the points.
        :param ys: The y coordinates of the points.
        :return: xs, ys, line, inliers: the outlier points, fitted line (a, b: y=ax+b), inliers
        """
        if len(xs) < self.detection_cfg["ransac_min_points"]:
            return xs, ys, None, None

        best_inliers = [[]]
        best_outliers = [[]]
        for _ in range(self.detection_cfg["ransac_iterations"]):
            idx = np.random.choice(len(xs), 2, replace=False)
            x1, x2 = xs[idx]
            y1, y2 = ys[idx]
            # The line is represented by the equation y = ax + b (+ avoid dividing by zero)
            if x2 == x1:
                continue

            a = (y2 - y1) / (x2 - x1)
            b = y1 - a * x1
            # Calculate the distance of each point to the line
            distances = np.abs(a * xs + b - ys) / np.sqrt(a ** 2 + 1)
            # Calculate the inliers
            threshold = self.detection_cfg["ransac_inliers_threshold"]
            inliers = np.where(distances < threshold)

            outliers = np.where(distances >= threshold)
            # If the number of inliers is greater than the best number of inliers so far, update the best inliers
            if len(inliers[0]) > len(best_inliers[0]):
                best_inliers = inliers
                best_outliers = outliers

        # Fit a line to the best inliers
        if len(best_inliers) > 0:
            line = np.polyfit(xs[best_inliers], ys[best_inliers], 1)
            inliers = np.array([xs[best_inliers], ys[best_inliers]]).T
            return xs[best_outliers], ys[best_outliers], line, inliers
        else:
            return xs, ys, None, None

    def calculate_goal(self, pillars, up=False) -> None:
        """
        Set the goal of OUR JOURNEY:). (policy differs based on the number of pillars)
        :param pillars: The map coordinates of the gate's pillars.
        :param up: Whether the garage is facing up or down.
        :return: None
        """
        # 2 pillars: we will try to get in front of the gate
        if len(pillars) == 2:
            # Set goal_type
            if not up:
                self.goal_type = self.detection_cfg['map']['goal_type']['two_pillars']
            else:
                self.goal_type = self.detection_cfg['map']['goal_type']['garage']

            # Set pillars
            pillar1 = pillars[0]
            pillar2 = pillars[1]

            # Calculate the center point between the two pillars
            center = ((pillar1[0] + pillar2[0]) // 2, (pillar1[1] + pillar2[1]) // 2)

            # Calculate perpendicular vector to the vector between the two pillars
            v = (pillar2[0] - pillar1[0], pillar2[1] - pillar1[1])
            gate_vector_magnitude = self.detection_cfg['map']['goal']['gate_vector_magnitude']
            v_perp = (-gate_vector_magnitude*v[1], gate_vector_magnitude*v[0])

            # Calculate the two points on the perpendicular vector which are on the line between the two pillars
            p1 = (int(center[0] + v_perp[0]), int(center[1] + v_perp[1]))
            p2 = (int(center[0] - v_perp[0]), int(center[1] - v_perp[1]))

            # Set the one with wanted (based on 'up' parameter) y coordinate as the goal
            if not up:
                self.goal_calculated = p1 if p1[1] < p2[1] else p2
            else:
                self.goal_calculated = p1 if p1[1] > p2[1] else p2

        # 1 pillar: we will try to get close to the pillar
        # 0 pillars: we will try to get closer to the closet point of the garage (yellow area on the map) if it exists
        elif len(pillars) == 1 or len(pillars) == 0:
            # Check if the robot object is assigned
            if self.robot is not None:
                # Get the coordinates of the robot and convert them to map coordinates
                x_robot = self.conv_real_to_map(self.robot.get_world_coordinates()[0], add=True)
                y_robot = self.conv_real_to_map(self.robot.get_world_coordinates()[1])
            else:
                raise ValueError("Robot not found.")

            # The reference object is the pillar
            if len(pillars) == 1:
                # Set goal_type
                self.goal_type = self.detection_cfg['map']['goal_type']['one_pillar']

                # Get map coordinates of the pillar
                ref_object_x = pillars[0][0]
                ref_object_y = pillars[0][1]
            # The reference object is the closest point of the garage
            else:
                # Set goal_type
                self.goal_type = self.detection_cfg['map']['goal_type']['garage']

                # Get the closest point of the garage (None if garage did not occur in the image)
                garage_id = self.detection_cfg['map']['id']['garage']
                closest_point = self.get_closest_point_on_map((x_robot, y_robot), garage_id)
                if closest_point is not None:
                    ref_object_x = closest_point[0]
                    ref_object_y = closest_point[1]
                else:
                    self.goal_calculated = None
                    return

            # Calculate the distance between the robot and the pillar
            distance = np.sqrt((ref_object_x - x_robot)**2 + (ref_object_y - y_robot)**2)

            # If the distance is smaller than the threshold, we are close enough to the reference object
            dist_threshold = self.detection_cfg['map']['goal']['min_distance_threshold']
            print("Distance  ", distance)
            print("Distance treshold (are we close?)", dist_threshold)                      
            if distance < dist_threshold:
                # In this case, fit the garage rectangle to the garage points
                p1, p2, p3, p4, first_line_len, second_line_len = self.fit_and_fill_garage_rectangle()

                # Decide where are the pillars of the gate
                if first_line_len > second_line_len:
                    pillar1 = p3
                    pillar2 = p4
                else:
                    pillar1 = p2
                    pillar2 = p3
                self.calculate_goal([pillar1, pillar2], up=True)
            # Otherwise, we will try to get closer to the pillar (point on the line between the robot and the pillar)
            else:
                # Get the vector between the robot and the reference object
                v = (ref_object_x - x_robot, ref_object_y - y_robot)

                # Make it a unit vector
                v = (v[0] / distance, v[1] / distance)

                # Calculate the point that is in the distance of the threshold from the pillar
                # Make the goal a bit closer
                dist_threshold = dist_threshold*0.6
                print("Distance treshold (go closer)", dist_threshold)                      
                x_goal = ref_object_x - v[0] * dist_threshold
                y_goal = ref_object_y - v[1] * dist_threshold
                self.goal_calculated = (int(x_goal), int(y_goal))
        else:
            raise ValueError("The gate has more than 2 pillars.")

    def correct_goal(self) -> None:
        """
        Correct the goal if it is not on the map or if it is in another object.
        Correcting the goal means setting it to the closest point available on the map. (ID = 0)
        :return: None
        """
        # Check if the goal is on the map
        if self.goal_calculated is not None:
            self.goal = self.get_closest_point_on_map(self.goal_calculated, 0)
        else:
            self.goal = None

    def get_closest_point_on_map(self, ref_point: tuple, id: int) -> tuple:
        """
        Get the closest point of the given ID on the map to the reference point.
        :param ref_point: The reference point.
        :param id: The ID of the wanted point.
        :return: The map coordinates of the closest point.
        """
        # Calculate the distance between the reference point and all points of the given ID on the map
        indices = np.where(self.world_map == id)
        distances = np.sqrt((indices[1] - ref_point[0])**2 + (indices[0] - ref_point[1])**2)

        # Get the index of the closest point if there is any
        if len(distances) == 0:
            return None 
        else:
            closest_index = np.argmin(distances)

        # Return the map coordinates of the closest point
        return indices[1][closest_index], indices[0][closest_index]

    def conv_real_to_map(self, realc, add=False) -> int:
        """
        Convert realc to map.
        :param realc: The real dims.
        :param add: If true, add the map center to the realc.
        """
        # Check if the realc is a number
        if realc is None:
            return None

        # Convert
        mapc = np.round(realc / self.resolution).astype(int)
        if add:
            mapc += self.world_map.shape[0] // 2
        return mapc

    def draw_restricted_area(self, center, size, convert=True, angle=0) -> None:
        """
        Draw restricted area around objects on the map.
        :param center: Center of the object.
        :param size: Size of the object.
        :param convert: If true, convert the real world parameters to map parameters.
        :param angle: Angle of the object. (radians)
        :return: None
        """
        if convert:
            # Convert real world parameters to map parameters
            x = self.conv_real_to_map(center[0], add=True)
            y = self.conv_real_to_map(center[1])
            size = self.conv_real_to_map(size)
            safety_margin = self.detection_cfg['map']['safety_margin_obstacles']
        else:
            x = center[0]
            y = center[1]
            safety_margin = self.detection_cfg['map']['safety_margin_garage']

        # Add half of the robot radius to the radius plus safety margin
        size += self.conv_real_to_map(self.robot.get_radius()) \
                  + self.conv_real_to_map(safety_margin)

        # Get id of the restricted area
        id = self.detection_cfg['map']['id']['restricted']

        # Draw the restricted area as a square
        restricted_area_type = self.detection_cfg['map']['restricted_area_type']
        if restricted_area_type == 'rectangle':
            self.draw_rectangle((x, y), size, id, angle)
        elif restricted_area_type == 'octagon':
            self.draw_octagon((x, y), size, id)

    def draw_rectangle(self, center, size, color, angle) -> None:
        """
        Draw a rectangle on the map.
        :param center: The center of the rectangle.
        :param size: The size of the rectangle.
        :param color: The color of the rectangle.
        :param angle: The angle of the rectangle. (radians)
        :return: None
        """
        # Calculate the coordinates of the four vertices of the rectangle
        vertices = []
        for i in range(4):
            x = int(center[0] + size * np.cos(2 * np.pi * i / 4 + angle + np.pi / 4))
            y = int(center[1] + size * np.sin(2 * np.pi * i / 4 + angle + np.pi / 4))
            vertices.append((x, y))

        # Draw the rectangle
        cv.fillConvexPoly(self.world_map, np.array(vertices), color)

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

    def draw_vision_cone(self) -> None:
        """
        Tha camera does not see the whole square in front of it, but only a triangle.
        This function draws restricted area around the vision cone of the camera.
        :return: None
        """
        # Get 8 map points of the vision cone from configuration file
        points = self.detection_cfg['map']['vision_cone']

        # Get id of the restricted area
        id = self.detection_cfg['map']['id']['restricted']

        # Get safety margin
        safety_margin = self.conv_real_to_map(self.detection_cfg['map']['safety_margin'])

        # Draw the restricted area as two triangles defined by the edges of the map and two lines (+ safety margin)
        p0 = points[0]
        p1 = points[1]
        p2 = [points[2][0] + safety_margin, points[2][1]]
        p3 = [points[3][0] + safety_margin, points[3][1]]
        cv.fillPoly(self.world_map, [np.array([p0, p1, p2, p3])], id)

        p0 = points[4]
        p1 = points[5]
        p2 = [points[6][0] - safety_margin, points[6][1]]
        p3 = [points[7][0] - safety_margin, points[7][1]]
        cv.fillPoly(self.world_map, [np.array([p0, p1, p2, p3])], id)

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

    def set_goal_calculated(self, goal_calculated):
        self.goal_calculated = goal_calculated

    def set_goal(self, goal):
        self.goal = goal

    def set_goal_type(self, goal_type):
        self.goal_type = goal_type

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

    def get_goal_calculated(self):
        return self.goal_calculated

    def get_goal(self):
        return self.goal

    def get_goal_type(self):
        return self.goal_type


if __name__ == "__main__":

    # Create a map
    import yaml
    import matplotlib.pyplot as plt

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)


    if True:
        from robot import Robot
        from detector import Detector
        # Set up robot -------------------------------
        rad = objects_cfg['robot']['radius']
        hei = objects_cfg['robot']['height']
        col = objects_cfg['robot']['color']

        rob = Robot(rad, col, 'black')
        print('robot object created')
        print('bumper initialized')

        rob.set_world_coordinates((0, 0))
        map.set_robot(rob)
        # --------------------------------------------

        img = rob.take_rgb_img()
        pc = rob.take_point_cloud()

        det = Detector(map, img, pc, detection_cfg, objects_cfg)
        det.process_rgb()
        det.process_point_cloud()

        map.fill_world_map()
        search_algorithm = detection_cfg['map']['search_algorithm']
        
        path = map.find_way((250, 0), tuple(map.get_goal()), search_algorithm)

        gar_coord = map.get_garage().get_world_coordinates()
        gar_map_x = map.conv_real_to_map(gar_coord[0], True)
        gar_map_y = map.conv_real_to_map(gar_coord[1])

        garage_points = np.array([gar_map_x, gar_map_y])

        # Save garage coordinates to file
        print('saving')
        np.save("garage_coordinates.npy", garage_points)

        from visualizer import Visualizer
        vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)
        vis.visualize_rgb()

    # Get garage dimensions
    garage_length = objects_cfg['garage']['length']
    garage_width = objects_cfg['garage']['width']

    # Convert garage dimensions to map coordinates
    garage_length_map = map.conv_real_to_map(garage_length)
    garage_width_map = map.conv_real_to_map(garage_width)

    # Generate garage points
    # garage_points =
    # generate_rectangle_points(garage_length_map, garage_width_map)
    # print(garage_points.shape)
    name = 'kratka.npy'
    # Load garage points from file
    #garage_points = np.load(name)
    print(name)
    print(garage_points.shape)

    # Fit a rectangle to the garage points
    p1, p2, p3, p4, len1, len2 = map.fit_rectangle(garage_points[0], garage_points[1], garage_length_map, garage_width_map)

    print(p1, p2, p3, p4, len1, len2)

    # Show the rectangle
    plt.scatter(garage_points[0], garage_points[1])
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], color='black')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], color='green')
    plt.plot([p3[0], p4[0]], [p3[1], p4[1]], color='pink')
    plt.plot([p4[0], p1[0]], [p4[1], p1[1]], color='yellow')
    plt.axis('equal')
    plt.show()
