import numpy as np
import cv2 as cv
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap

# just to get rid of annoying warnings
matplotlib.use('TkAgg')


class Map:
    def __init__(self, image, dimensions, resolution):
        self.image = image
        self.dimensions = dimensions
        self.resolution = resolution
        self.garage = None
        self.gate = None
        self.obstacles = []
        self.robot = None
        self.world_map = np.zeros((int(dimensions[1] / resolution), int(dimensions[0] / resolution)), np.uint8)

    def set_garage(self, garage):
        self.garage = garage

    def set_gate(self, gate):
        self.gate = gate

    def set_robot(self, robot):
        self.robot = robot

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def get_obstacles(self):
        return self.obstacles

    def get_gate(self):
        return self.gate

    def get_image(self):
        return self.image

    def fill_world_map(self, detection_cfg: dict, path=None):
        """
        Fill the world map with the found objects.
        :param detection_cfg: The configuration file for the object detection.
        """
        map_cfg = detection_cfg['map']

        # draw obstacles
        for obstacle in self.obstacles:
            # an obstacle is always a circle
            obst_id = map_cfg['id']['obstacle']
            x = int(obstacle.world_coordinates[0] / self.resolution) + self.world_map.shape[0] // 2
            y = int(obstacle.world_coordinates[1] / self.resolution)
            radius = int(obstacle.radius/self.resolution)
            cv.circle(self.world_map, (x, y), radius, obst_id, -1)

        # draw the robot
        if self.robot is not None:
            robot_id = map_cfg['id']['robot']
            x = int(self.robot.world_coordinates[0] / self.resolution) + self.world_map.shape[0] // 2
            y = int(self.robot.world_coordinates[1] / self.resolution)
            radius = int(self.robot.radius / self.resolution)
            cv.circle(self.world_map, (x, y), radius, robot_id, -1)

        # draw the garage
        if self.garage is not None:
            garage_id = map_cfg['id']['garage']
            x = int(self.garage.world_coordinates[0] / self.resolution) + self.world_map.shape[0] // 2
            y = int(self.garage.world_coordinates[1] / self.resolution)
            length = int(self.garage.width / self.resolution)
            width = int(self.garage.height / self.resolution)
            cv.rectangle(self.world_map, (x, y), (x+length, y+width), garage_id, -1)

        # draw the gate
        if self.gate is not None:
            gate_id = map_cfg['id']['gate']
            radius = int(self.gate.width / 2 / self.resolution)
            for slope in self.gate.world_coordinates:
                x = int(slope[0] / self.resolution) + self.world_map.shape[0] // 2
                y = int(slope[1] / self.resolution)
                cv.circle(self.world_map, (x, y), radius, gate_id, -1)

        if path != None:
            for i in range(len(self.world_map)):
                for j in range(len(self.world_map[0])):
                    if (i, j) in path:
                        self.world_map[i][j] = map_cfg['id']['path']

        # show 8-bit map
        cmap = ListedColormap(["white", "black", "yellow", "magenta", "red", "green"])
        self.plot_examples([cmap], self.world_map)

    def plot_examples(self, colormaps, data):
        """
        Helper function to plot data with associated colormap.
        """
        n = len(colormaps)
        fig, axs = plt.subplots(1, n, figsize=(10, 10), constrained_layout=True, squeeze=False)
        for [ax, cmap] in zip(axs.flat, colormaps):
            psm = ax.pcolormesh(data, cmap=cmap, rasterized=True, vmin=0, vmax=5)
            fig.colorbar(psm, ax=ax)
        plt.show()

    def visualize(self):
        """
        Visualize the map. This function is used for debugging and presenting purposes.
        """
        # draw a 2D map from above with a grid and the found objects using matplotlib

        # create a figure
        fig, ax = plt.subplots()

        # set the title
        ax.set_title('Map')

        # set the x and y limits so the robot is in the center of the bottom of the map
        robot_x = self.robot.world_coordinates[0]
        robot_y = self.robot.world_coordinates[1]
        ax.set_xlim(robot_x - self.dimensions[0] / 2, robot_x + self.dimensions[0] / 2)
        ax.set_ylim(robot_y, robot_y + self.dimensions[1])

        # set the x and y labels
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')

        # set the grid
        ax.grid()

        # draw the garage
        if self.garage is not None:
            ax.add_patch(Rectangle(self.garage.world_coordinates, self.garage.width, self.garage.height,
                                   angle=self.garage.orientation, color=self.garage.color))

        # draw the obstacles
        for obstacle in self.obstacles:
            # an obstacle is always a circle
            ax.add_patch(plt.Circle(obstacle.world_coordinates, obstacle.radius, color=obstacle.color))

        # draw the robot
        if self.robot is not None:
            ax.add_patch(plt.Circle(self.robot.world_coordinates, self.robot.radius, color=self.robot.color))

        # show the plot
        plt.show()

        # close the plot
        plt.close(fig)


class Obstacle:
    def __init__(self, radius, height, color, contour, world_coordinates=None):
        self.radius = radius
        self.height = height
        self.color = color
        self.contour = contour
        self.world_coordinates = world_coordinates

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def get_contour(self):
        return self.contour


class Garage:
    def __init__(self, length, width, height, color, contour):
        self.length = length
        self.width = width
        self.height = height
        self.color = color
        self.contour = contour
        self.world_coordinates = None
        self.orientation = None

    def set_orientation(self, orientation):
        self.orientation = orientation

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates


class Gate:
    def __init__(self, width, height, color, contours, slopes_distance, world_coordinates=None, orientation=None):
        # all this information is related to the slopes of the gate
        self.width = width
        self.height = height
        self.color = color
        self.contours = contours
        self.slopes_distance = slopes_distance
        self.world_coordinates = world_coordinates
        self.orientation = orientation
        self.num_slopes = len(contours)

    def set_orientation(self, orientation):
        self.orientation = orientation

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def get_num_slopes(self):
        return self.num_slopes


class Robot:
    def __init__(self, radius, height, color):
        self.radius = radius
        self.height = height
        self.color = color
        self.world_coordinates = None

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates


if __name__ == '__main__':
    # create a map
    map = Map(None, (5, 5), 0.01)

    # create a garage
    garage = Garage(0.6, 0.5, 0.4, None, 'yellow')
    garage.set_world_coordinates((0, 3))
    garage.set_orientation(0)

    # create a gate
    gate = Gate(0.10, 0.5, None, 'magenta', 0.5)
    gate.set_world_coordinates([(0, 3), (0.6, 3)])

    # create a robot
    robot = Robot(0.2, 0.4, 'black')
    robot.set_world_coordinates((0, 0))

    # create multiple obstacles
    obstacle = Obstacle(0.05, 0.4, 'red', None)
    obstacle.set_world_coordinates((-1, 1))

    obstacle2 = Obstacle(0.05, 0.4, 'blue', None)
    obstacle2.set_world_coordinates((1, 1))

    obstacle3 = Obstacle(0.05, 0.4, 'green', None)
    obstacle3.set_world_coordinates((0, 1))

    # add the objects to the map
    map.set_garage(garage)
    map.set_robot(robot)
    map.add_obstacle(obstacle)
    map.add_obstacle(obstacle2)
    map.add_obstacle(obstacle3)
    map.set_gate(gate)
    
    # visualize the map
    # map.visualize()

    # visualize the map in the world
    import yaml
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    map.fill_world_map(detection_cfg)

    cv.rectangle(map.world_map, (100, 100), (100+300, 100+50), 3, -1)

    from a_star2 import astar
    path = astar(map.world_map, (0, 250), (280, 280))
    map.fill_world_map(detection_cfg, path)