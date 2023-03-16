
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
    def __init__(self, width, height, color, contours, bounding_rects, slopes_distance, world_coordinates=None, orientation=None):
        # all this information is related to the slopes of the gate
        self.width = width
        self.height = height
        self.color = color
        self.contours = contours
        self.bounding_rect = bounding_rects
        self.slopes_distance = slopes_distance
        self.world_coordinates = world_coordinates
        self.orientation = orientation
        self.num_slopes = len(bounding_rects)

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

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def set_orientation(self, orientation):
        self.orientation = orientation

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

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_orientation(self):
        return self.orientation

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
