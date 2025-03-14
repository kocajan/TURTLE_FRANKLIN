import numpy as np


class Obstacle:
    def __init__(self, radius: float, height: float, color: str, contour: list, bounding_rect: list,
                 world_coordinates: list=None):
        """
        Obstacle objects are used to store information about the obstacles detected by the robot.
        :param radius: Radius of the obstacle.
        :param height: Height of the obstacle.
        :param color: Color of the obstacle.
        :param contour: Contour of the obstacle.
        :param bounding_rect: Bounding rectangle of the obstacle.
        :param world_coordinates: World coordinates of the obstacle.
        """
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
    def __init__(self, length: float, width: float, height: float, color: str, contours: list):
        """
        Garage objects are used to store information about the garage detected by the robot.
        :param length: Length of the garage.
        :param width: Width of the garage.
        :param height: Height of the garage.
        :param color: Color of the garage.
        :param contours: Contours of the garage.
        """
        self.length = length
        self.width = width
        self.height = height
        self.color = color
        self.contours = contours
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

    def set_contours(self, contours):
        self.contours = contours

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

    def get_contours(self):
        return self.contours

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_orientation(self):
        return self.orientation


class Gate:
    def __init__(self, width: float, height: float, color: str, contours: list, bounding_rects: list,
                 pillars_distance: float, lowest_points: list, garage_dimensions_lwh: list,
                 world_coordinates: list=None, orientation: float=None, orientation_rgb: float=None, center: list=None):
        """
        Gate objects are used to store information about the gate detected by the robot.
        All this information is related to the pillars of the gate.
        :param width: Width of the gate.
        :param height: Height of the gate.
        :param color: Color of the gate.
        :param contours: Contours of the gate.
        :param bounding_rects: Bounding rectangles of the gate.
        :param pillars_distance: Distance between the pillars of the gate.
        :param lowest_points: Lowest points of the pillars of the gate.
        :param garage_dimensions_lwh: Dimensions of the garage (length, width, height).
        :param world_coordinates: World coordinates of the gate.
        :param orientation: Orientation of the gate.
        :param orientation_rgb: Orientation of the gate (RGB).
        :param center: Center of the gate.
        """
        self.width = width
        self.height = height
        self.color = color
        self.contours = contours
        self.bounding_rects = bounding_rects
        self.pillars_distance = pillars_distance
        self.lowest_points = lowest_points
        self.garage_dimensions_lwh = garage_dimensions_lwh
        self.world_coordinates = world_coordinates
        self.orientation = orientation
        self.orientation_rgb = orientation_rgb
        self.center = center
        self.num_pillars = len(bounding_rects)

    def calculate_orientation(self) -> None:
        """
        Calculate the orientation of the gate from the analysed depth picture (world coordinates).
        :return: None
        """
        if self.num_pillars != 2:
            return

        # Get the real world coordinates of the pillars
        x1, y1 = self.world_coordinates[0]
        x2, y2 = self.world_coordinates[1]

        # Calculate the angle between the gate and the horizontal axis
        self.orientation = np.arctan2(y1 - y2, x1 - x2) if y1 >= y2 else np.arctan2(y2 - y1, x2 - x1)

    def calculate_orientation_rgb(self) -> None:
        """
        Calculate the orientation of the gate from the analysed RGB picture.
        :return: None
        """

        if self.num_pillars != 2:
            return

        # Get the lowest points of the pillars
        x1, y1 = self.lowest_points[0]
        x2, y2 = self.lowest_points[1]

        # Calculate center of the gate
        self.center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

        # Calculate the angle between the gate and the horizontal axis
        self.orientation_rgb = np.arctan2(y1 - y2, x1 - x2)

    def remove_pillar(self, pillar) -> None:
        """
        Remove a pillar from the gate.
        :param pillar: the pillar to remove
        :return: None
        """
        tmp = []
        for pill in self.bounding_rects:
            if pill is not pillar:
                tmp.append(pill)
        self.bounding_rects = tmp
        self.num_pillars = len(self.bounding_rects)

    # Setters
    def set_width(self, width):
        self.width = width

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_contours(self, contours):
        self.contours = contours

    def set_bounding_rects(self, bounding_rects):
        self.bounding_rects = bounding_rects

    def set_pillars_distance(self, pillars_distance):
        self.pillars_distance = pillars_distance

    def set_lowest_points(self, lowest_points):
        self.lowest_points = lowest_points

    def set_garage_dimensions_lwh(self, garage_dimensions_lwh):
        self.garage_dimensions_lwh = garage_dimensions_lwh

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def set_orientation(self, orientation):
        self.orientation = orientation

    def set_orientation_rgb(self, orientation_rgb):
        self.orientation_rgb = orientation_rgb

    def set_center(self, center):
        self.center = center

    def set_num_pillars(self, num_pillars):
        self.num_pillars = num_pillars

    # Getters
    def get_width(self):
        return self.width

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_contours(self):
        return self.contours

    def get_bounding_rects(self):
        return self.bounding_rects

    def get_pillars_distance(self):
        return self.pillars_distance

    def get_lowest_points(self):
        return self.lowest_points

    def get_garage_dimensions_lwh(self):
        return self.garage_dimensions_lwh

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_orientation(self):
        return self.orientation

    def get_orientation_rgb(self):
        return self.orientation_rgb

    def get_center(self):
        return self.center

    def get_num_pillars(self):
        return self.num_pillars
