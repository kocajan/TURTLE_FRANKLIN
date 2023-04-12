from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate

GLOBAL_STOP = False


class robot:
    # Names bumpers and events
    #bumper_names = ['LEFT', 'CENTER', 'RIGHT']
    #state_names = ['RELEASED', 'PRESSED']

    def __init__(self, radius, height, color):
        self.turtle = Turtlebot() #this only project used to drive the robot
        self.radius = radius
        self.height = height
        self.color = color
        self.world_coordinates = None

    def get_RGB_img(self):
        """
        Take RGB photo and return it as numpy array.
        :return: RGB photo as numpy array.
        """
        # Wait for the camera to setup
        self.turtle.wait_for_rgb_image()

        # Capture BGR image
        bgr = self.turtle.get_rgb_image()

        # Convert BGR to RGB and save image
        rgb = bgr[..., ::-1]
        return rgb

    def get_point_cloud(self):
        """
        Get point cloud from the robot.
        :return: Point cloud as numpy array.
        """
        # Wait for the camera to setup
        self.turtle.wait_for_point_cloud()

        # Capture point cloud
        pc = self.turtle.get_point_cloud()
        return pc

    def stop_motors(self):
        self.turtle.cmd_velocity(linear = 0)
        self.turtle.cmd_velocity(angular = 0)

    def bumper_cb(self, msg):
        """Bumber callback."""
        # msg.bumper stores the id of bumper 0:LEFT, 1:CENTER, 2:RIGHT
        #bumper = bumper_names[msg.bumper]

        # msg.state stores the event 0:RELEASED, 1:PRESSED
        #state = state_names[msg.state]
        # Print the event
        #print('{} bumper {}'.format(bumper, state))
        GLOBAL_STOP = True
        self.stop_motors()

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

    def main(self):
        # Register bumper callback
        self.turtle.register_bumper_event_cb(self.bumper_cb)

        # Do something, the program would end otherwise
        rate = Rate(1)
        while not self.turtle.is_shutting_down():
            rate.sleep()

        #TODO je potreba predavat turtle jako parametr do move

        # hledani branky nejakej loop {}
        # cesta k bodu U BRANKY loop {}
        # parkovani loop {}


if __name__ == '__main__':
    my_robot = robot()
    my_robot.main()
