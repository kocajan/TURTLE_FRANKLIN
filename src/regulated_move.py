# co je potreba?? pozice x a y podle odometrie, to vemem z move
# spocitat error vuci ceste na konkretnim y
# tento error x ka z odometrie a xka z cesty narveme feedackem do regulatoru


#odometrie je v metrech
# odometrie je + dopredu a - dozadu

# ten error budeme cpat jen do rotace??
# prvni zkusime jednpoduchy P regulator. Error x od x odometrie get_odometry[1], ten nacpeme nejak znasobeny do rotace?

import sys
import numpy as np
import math
import yaml
from rospy import Rate

class regulated_move:
    def __init__(self):
        pass

    def go_straight(self, length, dir):
        slow_start_cnt = 0
        slowdown = False
        odometry_x_y = []
        # path_integrated = list()
        self.odometry_hard_reset()
        while not self.robot.is_shutting_down() and not self.robot.get_stop():
            # print(slow_start_cnt)
            if slow_start_cnt < 1.0 and not slowdown:
                slow_start_cnt += 0.15
            elif slowdown and slow_start_cnt >= 0.25:  # 0.17
                slow_start_cnt -= 0.1

            odometry_x_y = self.robot.get_odometry()[:1]
            if abs(odometry_x_y[0]) >= (length / 100):
                break
            self.robot.cmd_velocity(linear=(0.2 * dir) * slow_start_cnt)  # TODO add as yaml const
            self.rate.sleep()
            # print((abs(odometry_x_y[0])/2), ((length/100)/2))
            if (abs(odometry_x_y[0])) > ((length / 100) / 1.8):  # /1.3
                slowdown = True
        # path_integrated.append(odometry_x_y[0])
        # path_integrated.append(odometry_x_y[1])
        return None  # list(odometry_x_y[0], odometry_x_y[1])

    def odometry_hard_reset(self):
        """
        Hard reset the robot's odometry.
        """
        self.robot.reset_odometry()
        self.robot.wait_for_odometry()
        self.robot.reset_odometry()

    def rotate_degrees_no_compensation(self, degrees, speed):
        self.odometry_hard_reset()
        if abs(degrees) > 40:
            offset = 0.08
        else:
            offset = 0

        if degrees > 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                if act_rot >= goal-offset or abs(act_rot - prev_rot) > 1:
                    break
                self.robot.cmd_velocity(angular = speed)
                prev_rot = act_rot

        elif degrees < 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                # print(act_rot, prev_rot)
                if act_rot <= goal+offset or abs(act_rot - prev_rot) > 1:
                    break
                self.robot.cmd_velocity(angular = -speed)
                prev_rot = act_rot

    def go(self, path):

        # reset odometry??
        # kdyz jsme v dostatecne blizkosti pozadovaneho setpointu tak posunem index v path
        P = 1
        error = 0
        setpoint = 1 # protoze na prvnim stojime
        goal = path[-1][0]

        # Example array
        odometry_cm = np.array()

        # pri jizde dopredu se nemeni get_odometry[1] - odometry[1] je ve svete robota x
        # odometry[0] je ve
        # takze pro pocitani erroru je pro nas dulezite odometry[]

        # pro index v path plati ze bereme zaokrouhlenou get_odometry[0]
        # pro error jizdy plati ze bereme get_odometry[1]

        while(True):
            #if near_to_goal :
            #    break
            odometry_cm = self.robot.get_odometry()*100 # take actual odometry in cm [x,y,rot]

            error = path[round(odometry_cm[0] + 1)][0] - odometry_cm[1]
            self.robot.cmd_velocity(linear=0.1, angular=P*error)
            self.rate.sleep()

            #if dostatecne blizko
            #    setpoint += 1

        # pocitame error pri kazde iteraci