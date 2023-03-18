import numpy as np
import cv2 as cv


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
    def fill_world_map(self):
        """
        Fill the world map with the objects which were found by detector.
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

    def conv_real_to_map(self, realc, add=False):
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