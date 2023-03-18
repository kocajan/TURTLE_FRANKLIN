import numpy as np


class Obstacle:
    def __init__(self, radius, height, color, contour, bounding_rect, world_coordinates=None):
        self.radius = radius
        self.height = height
        self.color = color
        self.contour = contour
        self.bounding_rect = bounding_rect
        self.world_coordinates = world_coordinates

    # Setters
    def set_radius(self, radius):
        self.radius = radius

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_contour(self, contour):
        self.contour = contour

    def set_bounding_rect(self, bounding_rect):
        self.bounding_rect = bounding_rect

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    # Getters
    def get_radius(self):
        return self.radius

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_contour(self):
        return self.contour

    def get_bounding_rect(self):
        return self.bounding_rect

    def get_world_coordinates(self):
        return self.world_coordinates


class Garage:
    def __init__(self, length, width, height, color, contour):
        self.length = length
        self.width = width
        self.height = height
        self.color = color
        self.contour = contour
        self.world_coordinates = None
        self.orientation = None

    # Setters
    def set_length(self, length):
        self.length = length

    def set_width(self, width):
        self.width = width

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_contour(self, contour):
        self.contour = contour

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def set_orientation(self, orientation):
        self.orientation = orientation

    # Getters
    def get_length(self):
        return self.length

    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_contour(self):
        return self.contour

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_orientation(self):
        return self.orientation


class Gate:
    def __init__(self, width, height, color, contours, bounding_rects, slopes_distance, lowest_points,
                 world_coordinates=None, orientation=None, center=None):
        # all this information is related to the slopes of the gate
        self.width = width
        self.height = height
        self.color = color
        self.contours = contours
        self.bounding_rect = bounding_rects
        self.slopes_distance = slopes_distance
        self.lowest_points = lowest_points
        self.world_coordinates = world_coordinates
        self.orientation = orientation
        self.center = center
        self.num_slopes = len(bounding_rects)

    def calculate_orientation(self) -> None:
        """
        Calculate the orientation of the gate.
        :return: None
        """

        if self.num_slopes != 2:
            return

        # Get the lowest points of the slopes
        x1, y1 = self.lowest_points[0]
        x2, y2 = self.lowest_points[1]

        # Calculate center of the gate
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

        # Calculate the angle between the gate and the horizontal axis
        self.orientation = np.arctan2(y1 - y2, x1 - x2)

    # Setters
    def set_width(self, width):
        self.width = width

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_contours(self, contours):
        self.contours = contours

    def set_bounding_rect(self, bounding_rects):
        self.bounding_rect = bounding_rects

    def set_slopes_distance(self, slopes_distance):
        self.slopes_distance = slopes_distance

    def set_lowest_points(self, lowest_points):
        self.lowest_points = lowest_points

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def set_orientation(self, orientation):
        self.orientation = orientation

    def set_center(self, center):
        self.center = center

    def set_num_slopes(self, num_slopes):
        self.num_slopes = num_slopes

    # Getters
    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_contours(self):
        return self.contours

    def get_bounding_rect(self):
        return self.bounding_rect

    def get_slopes_distance(self):
        return self.slopes_distance

    def get_lowest_points(self):
        return self.lowest_points

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_orientation(self):
        return self.orientation

    def get_center(self):
        return self.center

    def get_num_slopes(self):
        return self.num_slopes


class Robot:
    def __init__(self, radius, height, color):
        self.radius = radius
        self.height = height
        self.color = color
        self.world_coordinates = None

    # Setters
    def set_radius(self, radius):
        self.radius = radius

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    # Getters
    def get_radius(self):
        return self.radius

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_world_coordinates(self):
        return self.world_coordinates
