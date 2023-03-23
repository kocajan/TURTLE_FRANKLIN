from robolab_turtlebot import Turtlebot, Rate, get_time
import numpy as np
import math
from enum import Enum
import yaml
import single_mv

class move:
    move_coords = list(list())  #2D array of moving coords

    def __init__(self, coords):
        self.detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
        self.move_coords = coords
        self.turtle = Turtlebot()
        self.rate = Rate(10) # co dela tahle funkce?

    def execute_move(self):
        # d
        pass

    #exports nonlinear path, to linear
    def non_to_linear_path(self, non_linear_path):
        linear_path = list()
        return linear_path

    def move_sequence(self):
        for x in self.move_coords:

    # length in centimeters
    def go_straight(self, length, dir):
        slow_start_cnt = 0
        slowdown = False
        odometry_x_y = []
        #path_integrated = list()
        self.odometry_hard_rst()
        while not self.turtle.is_shutting_down():
            #print(slow_start_cnt)
            if(slow_start_cnt < 1.0 and not slowdown):
                slow_start_cnt = slow_start_cnt + 0.05
            elif(slowdown and slow_start_cnt >= 0.17):
                slow_start_cnt = slow_start_cnt - 0.05

            odometry_x_y = self.turtle.get_odometry()[:1]
            if(abs(odometry_x_y[0]) >= (length/100)):
                break
            self.turtle.cmd_velocity(linear=(0.2*dir)*slow_start_cnt) # TODO add as yaml const
            self.rate.sleep()
            print((abs(odometry_x_y[0])/2), ((length/100)/2))
            if((abs(odometry_x_y[0])) > ((length/100)/1.3)):
                slowdown = True
        #path_integrated.append(odometry_x_y[0])
        #path_integrated.append(odometry_x_y[1])
        return list(odometry_x_y[0], odometry_x_y[1])

    def compensate_straight_integration_drift(self):
        additional_rotation = 0
        #pricitej zataceci error podle integrace k jiz planovanemu zatoceni
        return additional_rotation

    def odometry_hard_rst(self):
        self.turtle.reset_odometry()
        self.turtle.wait_for_odometry()
        self.turtle.reset_odometry()

    # this is compensation for error drift after rotation
    def compensation(self, direction):
        #TODO move compensation constant to yaml

        self.odometry_hard_rst()
        if(direction == -1):
            while not self.turtle.is_shutting_down():
                self.turtle.cmd_velocity(angular = 0.4)
                act_rot = self.turtle.get_odometry()[2]
                print(act_rot)
                if(act_rot >= 0.08):
                    break

        elif(direction == 1):
            while not self.turtle.is_shutting_down():
                self.turtle.cmd_velocity(angular = -0.4)
                act_rot = self.turtle.get_odometry()[2]
                print(act_rot)
                if(act_rot <= -0.08):
                    break
                
    # degree input range have to be +-180 deg
    def rotate_degrees(self, degrees):
        self.odometry_hard_rst()

        if(degrees > 0):
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.turtle.is_shutting_down():
                act_rot = self.turtle.get_odometry()[2]
                print(act_rot, prev_rot)
                if(act_rot >= goal or abs(act_rot-prev_rot) > 1):
                    break
                self.turtle.cmd_velocity(angular = 0.4)
                prev_rot = act_rot
            self.compensation(1)
  
        elif(degrees < 0):
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.turtle.is_shutting_down():
                act_rot = self.turtle.get_odometry()[2]
                print(act_rot, prev_rot)
                if(act_rot <= goal or abs(act_rot - prev_rot) > 1):
                    break
                self.turtle.cmd_velocity(angular = -0.4)
                prev_rot = act_rot
            self.compensation(-1)
        

tmp = move(None)
#tmp.rotate_degrees(-60)
tmp.go_straight(100,1)
#tmp.rotate_degrees(-180)
