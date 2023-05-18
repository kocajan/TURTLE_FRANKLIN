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
    def __init__(self, rob):
        self.robot = rob

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
if __name__ == '__main__':
    from robot import Robot
    path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (269, 20), (269, 21), (269, 22), (269, 23), (269, 24), (269, 25), (269, 26), (269, 27), (269, 28), (269, 29), (269, 30), (269, 31), (269, 32), (269, 33), (269, 34), (269, 35), (269, 36), (269, 37), (269, 38), (270, 39), (271, 40), (272, 41), (273, 42), (274, 43), (275, 44), (276, 45), (277, 46), (278, 47), (279, 48), (279, 49), (279, 50), (279, 51), (279, 52), (279, 53), (279, 54), (279, 55), (279, 56), (279, 57), (279, 58), (279, 59), (279, 60), (279, 61), (279, 62), (279, 63), (279, 64), (279, 65), (278, 66), (277, 67), (276, 68), (275, 69), (274, 70), (273, 71), (272, 72), (271, 73), (270, 74), (269, 75), (269, 76), (269, 77), (269, 78), (269, 79), (269, 80), (269, 81), (269, 82), (269, 83), (269, 84), (269, 85), (269, 86), (269, 87), (269, 88), (269, 89), (269, 90), (269, 91), (269, 92), (269, 93), (269, 94), (269, 95), (269, 96), (269, 97), (269, 98), (269, 99), (269, 100), (269, 101), (269, 102), (269, 103), (269, 104), (269, 105), (269, 106), (269, 107), (269, 108)]
    
    detection_cfg = yaml.safe_load(open('../conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('../conf/objects.yaml', 'r'))
    robot_cfg = objects_cfg['robot']
    rob = Robot(robot_cfg['radius'], robot_cfg['height'], robot_cfg['color'])
    rob.set_world_coordinates(robot_cfg['world_coordinates'])

    tmp = regulated_move(rob)
    tmp.go(path)