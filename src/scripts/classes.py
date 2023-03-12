import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt

from matplotlib.patches import Rectangle


class Map:
    def __init__(self, dimensions):
        self.dimensions = dimensions
        self.garage = None
        self.gate = None
        self.obstacles = []
        self.robot = None

    def set_garage(self, garage):
        self.garage = garage

    def set_gate(self, gate):
        self.gate = gate

    def set_robot(self, robot):
        self.robot = robot

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

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


class Garage:
    def __init__(self, length, width, height, color):
        self.length = length
        self.width = width
        self.height = height
        self.color = color
        self.contour = None
        self.world_coordinates = None
        self.orientation = None

    def set_contour(self, contour):
        self.contour = contour

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
    map = Map((5, 5))

    # create a garage
    garage = Garage(0.6, 0.5, 0.4, 'yellow', 'magenta')
    garage.set_world_coordinates((-1, 3))
    garage.set_orientation(32.5)

    # create a robot
    robot = Robot(0.2, 0.4, 'black')
    robot.set_world_coordinates((0, 0))

    # create multiple obstacles
    obstacle = Obstacle(0.05, 0.4, 'red')
    obstacle.set_world_coordinates((1, 3))

    obstacle2 = Obstacle(0.05, 0.4, 'blue')
    obstacle2.set_world_coordinates((-0.5, 2.5))

    obstacle3 = Obstacle(0.05, 0.4, 'green')
    obstacle3.set_world_coordinates((1, 1))

    # add the objects to the map
    map.set_garage(garage)
    map.set_robot(robot)
    map.add_obstacle(obstacle)
    map.add_obstacle(obstacle2)
    map.add_obstacle(obstacle3)

    # visualize the map
    map.visualize()

