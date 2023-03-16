from robolab_turtlebot import Turtlebot, Rate, get_time
import numpy as np
import math
from enum import Enum
import yaml


class move:
    move_coords = list(list())
    

    def __init__(self, coords):
        self.detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
        self.move_coords = coords
        self.turtle = Turtlebot()
        self.rate = Rate(10)

    def rotate_degrees(self, degrees):
        self.turtle.reset_odometry()
        temp = self.turtle.get_odometry()[2]
        goal = np.degrees(temp) + degrees
        goal_radians = np.radians(goal)

        radians = np.radians(degrees) 
        print(goal_radians)
        while not self.turtle.is_shutting_down() :
            if(degrees < 0):
                self.turtle.cmd_velocity(angular = -0.5)
            if(degrees > 0):
                self.turtle.cmd_velocity(angular = 0.5)

            actual_rotation = self.turtle.get_odometry()[2]
            print(goal_radians, actual_rotation)
            self.rate.sleep()

    def test(self):
        t = get_time()

        '''while get_time() - t < 10:
            self.turtle.cmd_velocity(linear=0.1)
            self.rate.sleep()
'''
        #self.turtle.reset_odometry()
        '''t = get_time()
        while not self.turtle.is_shutting_down():
            if get_time() - t > 0.5*6.28:
                break
            self.turtle.cmd_velocity(angular = -2)
            self.rate.sleep() 
'''
        self.turtle.reset_odometry()
        while not self.turtle.is_shutting_down():
            linear_motion = list() # bude to list linearnich motionu
            var = self.turtle.get_odometry()
            #if (var[0] >= 1 or var[1] >= 1):
            #    break
            print(var[0], var[1], var[2])
            #direction = np.sign(np.random.rand() - 0.5)
            self.turtle.cmd_velocity(angular=-0.3)
            #turtle.cmd_velocity(linear=0.1)
            self.rate.sleep()
        

tmp = move(None)
tmp.rotate_degrees(90)
for i in range(0):
    tmp.rotate_degrees(45)
#tmp.test()