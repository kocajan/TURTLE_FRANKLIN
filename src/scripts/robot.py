from __future__ import print_function
from robolab_turtlebot import Turtlebot, Rate
from move import move

#GLOBAL_STOP = False

<<<<<<< HEAD:src/scripts/robot.py
class robot(Turtlebot):
=======

class robot:
>>>>>>> d6dc65d66e2c19e564b2753d0d23db89bc565217:robot.py
    # Names bumpers and events
    #bumper_names = ['LEFT', 'CENTER', 'RIGHT']
    #state_names = ['RELEASED', 'PRESSED']

    def __init__(self, radius, height, color, rgb=True, depth=False, pc=True):
        super().__init__(rgb=rgb,
                        depth=depth,
                        pc=pc)

        #self.turtle = Turtlebot() #this only project used to drive the robot
        self.radius = radius
        self.height = height
        self.color = color
        self.world_coordinates = None
        self.GLOBAL_STOP = False

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
<<<<<<< HEAD:src/scripts/robot.py
        self.cmd_velocity(linear = 0)
        self.cmd_velocity(angular = 0)
=======
        self.turtle.cmd_velocity(linear = 0)
        self.turtle.cmd_velocity(angular = 0)
>>>>>>> d6dc65d66e2c19e564b2753d0d23db89bc565217:robot.py

    def bumper_cb(self, msg):
        """Bumber callback."""
        # msg.bumper stores the id of bumper 0:LEFT, 1:CENTER, 2:RIGHT
        #bumper = bumper_names[msg.bumper]

        # msg.state stores the event 0:RELEASED, 1:PRESSED
        #state = state_names[msg.state]
        # Print the event
        #print('{} bumper {}'.format(bumper, state))
        self.GLOBAL_STOP = True
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
        self.register_bumper_event_cb(self.bumper_cb)

        # Do something, the program would end otherwise
        rate = Rate(1)
        while not self.is_shutting_down():
            rate.sleep()

        #TODO je potreba predavat turtle jako parametr do move

        # hledani branky nejakej loop {}
        # cesta k bodu U BRANKY loop {}
        # parkovani loop {}
    def get_GLOBAL_STOP(self):
        return self.GLOBAL_STOP

if __name__ == '__main__':
    my_robot = robot(1, 1, 1)
    my_robot.main()
    path =  [(250, 0), (251, 1), (252, 2), (253, 3), (253, 4), (253, 5), (253, 6), (253, 7), (253, 8), (253, 9), (253, 10), (253, 11), (253, 12), (253, 13), (253, 14), (253, 15), (253, 16), (253, 17), (253, 18), (253, 19), (253, 20), (253, 21), (253, 22), (253, 23), (253, 24), (253, 25), (253, 26), (253, 27), (253, 28), (253, 29), (253, 30), (253, 31), (253, 32), (253, 33), (253, 34), (253, 35), (253, 36), (253, 37), (253, 38), (253, 39), (253, 40), (253, 41), (253, 42), (253, 43), (253, 44), (253, 45), (253, 46), (253, 47), (253, 48), (253, 49), (253, 50), (253, 51), (253, 52), (253, 53), (253, 54), (253, 55), (253, 56), (253, 57), (253, 58), (253, 59), (253, 60), (253, 61), (253, 62), (253, 63), (253, 64), (253, 65), (253, 66), (253, 67)]
    tmp = move(my_robot, path)
    #tmp.move_sequence()
    tmp.execute_move()

