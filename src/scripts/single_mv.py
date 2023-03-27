
class single_mv:
    move_coords = list(list())  #2D array of moving coords
    def __init__(self, degrees, distance):
        self.rotation_val = degrees
        self.distance_val = distance

    def get_rotation(self):
        return self.rotation_val
    def get_go_distance(self):
        return self.rotation_val
