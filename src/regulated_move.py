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
        self.rate = Rate(10)

    def odometry_hard_reset(self):
        """
        Hard reset the robot's odometry.
        """
        self.robot.reset_odometry()
        self.robot.wait_for_odometry()
        self.robot.reset_odometry()


    def calculate_distance(self, pt1, pt2):
        return np.linalg.norm(np.array(pt2) - np.array(pt1))

    def substract_offset(self, path, x_offset):
        new_path = list()
        for item in path:
            new_path.append( (item[0]-x_offset, item[1]) )
        return new_path

    def go(self, path):
        x_offset = 269
        path = self.substract_offset(path, x_offset)
        P = 0
        I = 0
        sum = 0
        setpoint_idx = 10
        setpoint = path[setpoint_idx] # protoze na prvnim stojime

        prev_odometry_values = []

        goal = path[-1][0]

        self.odometry_hard_reset()

        while not self.robot.is_shutting_down() and not self.robot.get_stop():

            odometry_cm = self.robot.get_odometry()[:2]*100 # take actual odometry in cm [x,y,rot]

            prev_odometry_values.append(odometry_cm)

            if len(prev_odometry_values) > 100:
                prev_odometry_values.pop(0)

            error = self.calculate_error(setpoint, odometry_cm[::-1], prev_odometry_values)
            print("Error: ", error)

            if abs(sum) > 10:
                sum = 0
            else:
                sum = error + sum

            if self.calculate_distance(odometry_cm[::-1], setpoint) < 5:
                setpoint_idx += 5
                setpoint = path[setpoint_idx]

            self.robot.cmd_velocity(linear=0.1, angular=P*error + I*sum)
            self.rate.sleep()

    def calculate_error(self, setpoint, current_odometry_value, previous_odometry_values):
        """
        Calculate the error for the robot's regulator.
        It is represented as the difference between velocity vector and the vector from the robot to the goal.
        The odometry value is in the form of (y, x).
        :param setpoint: Setpoint for the robot's regulator.
        :param current_odometry_value: Current odometry value.
        :param previous_odometry_values: List of previous odometry values.
        :return: Error for the robot's regulator. (the angle)
        """
        # Calculate the difference between the current odometry value and the previous odometry value
        # Take the n-th previous odometry value (It will represent the velocity vector)
        velocity_vector = self.calculate_difference(current_odometry_value, previous_odometry_values, n=1)

        # Calculate the vector from the robot to the goal.
        vector_to_goal = np.array(setpoint) - np.array(current_odometry_value)

        # Calculate the angle between the vector from the robot to the goal and the robot's velocity vector
        # The angle is in radians
        angle = math.atan2(vector_to_goal[0], vector_to_goal[1]) - math.atan2(velocity_vector[0], velocity_vector[1])

        return angle

    @staticmethod
    def calculate_difference(current_odometry_value, previous_odometry_values, n=1):
        """
        Calculate the difference between the current odometry value and the previous odometry value.
        Take the n-th previous odometry value.
        The odometry value is in the form of (y, x).
        :param current_odometry_value: Current odometry value.
        :param previous_odometry_values: List of previous odometry values.
        :param n: Take the n-th previous odometry value.
        :return: Difference between the current odometry value and the previous odometry value.
        """
        return current_odometry_value - previous_odometry_values[-n]

if __name__ == '__main__':

    from robot import Robot
    path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (269, 20), (269, 21), (269, 22), (269, 23), (269, 24), (269, 25), (269, 26), (269, 27), (269, 28), (269, 29), (269, 30), (269, 31), (269, 32), (269, 33), (269, 34), (269, 35), (269, 36), (269, 37), (269, 38), (270, 39), (271, 40), (272, 41), (273, 42), (274, 43), (275, 44), (276, 45), (277, 46), (278, 47), (279, 48), (279, 49), (279, 50), (279, 51), (279, 52), (279, 53), (279, 54), (279, 55), (279, 56), (279, 57), (279, 58), (279, 59), (279, 60), (279, 61), (279, 62), (279, 63), (279, 64), (279, 65), (278, 66), (277, 67), (276, 68), (275, 69), (274, 70), (273, 71), (272, 72), (271, 73), (270, 74), (269, 75), (269, 76), (269, 77), (269, 78), (269, 79), (269, 80), (269, 81), (269, 82), (269, 83), (269, 84), (269, 85), (269, 86), (269, 87), (269, 88), (269, 89), (269, 90), (269, 91), (269, 92), (269, 93), (269, 94), (269, 95), (269, 96), (269, 97), (269, 98), (269, 99), (269, 100), (269, 101), (269, 102), (269, 103), (269, 104), (269, 105), (269, 106), (269, 107), (269, 108)]
    path = [(269, 75), (269, 76), (269, 77), (269, 78), (269, 79), (269, 80), (269, 81), (269, 82), (269, 83), (269, 84), (269, 85), (269, 86), (269, 87), (269, 88), (269, 89), (269, 90), (269, 91), (269, 92), (269, 93), (269, 94), (269, 95), (269, 96), (269, 97), (269, 98), (269, 99), (269, 100), (269, 101), (269, 102), (269, 103), (269, 104), (269, 105), (269, 106), (269, 107), (269, 108)]
    path = [(269, 0), (269, 1), (269, 2), (269, 3), (269, 4), (269, 5), (269, 6), (269, 7), (269, 8), (269, 9), (269, 10), (269, 11), (269, 12), (269, 13), (269, 14), (269, 15), (269, 16), (269, 17), (269, 18), (269, 19), (269, 20), (269, 21), (269, 22), (269, 23), (269, 24), (269, 25), (269, 26), (269, 27), (269, 28), (269, 29), (269, 30), (269, 31), (269, 32), (269, 33), (269, 34), (269, 35), (269, 36), (269, 37), (269, 38), (269, 39), (269, 40), (269, 41), (269, 42), (269, 43), (269, 44), (269, 45), (269, 46), (269, 47), (269, 48), (269, 49), (269, 50), (269, 51), (269, 52), (269, 53), (269, 54), (269, 55), (269, 56), (269, 57), (269, 58), (269, 59), (269, 60), (269, 61), (269, 62), (269, 63), (269, 64), (269, 65), (269, 66), (269, 67), (269, 68), (269, 69), (269, 70), (269, 71), (269, 72), (269, 73), (269, 74), (269, 75), (269, 76), (269, 77), (269, 78), (269, 79), (269, 80), (269, 81), (269, 82), (269, 83), (269, 84), (269, 85), (269, 86), (269, 87), (269, 88), (269, 89), (269, 90), (269, 91), (269, 92), (269, 93), (269, 94), (269, 95), (269, 96), (269, 97), (269, 98), (269, 99), (269, 100), (269, 101), (269, 102), (269, 103), (269, 104), (269, 105), (269, 106), (269, 107), (269, 108), (269, 109), (269, 110), (269, 111), (269, 112), (269, 113), (269, 114), (269, 115), (269, 116), (269, 117), (269, 118), (269, 119), (269, 120), (269, 121), (269, 122), (269, 123), (269, 124), (269, 125), (269, 126), (269, 127), (269, 128), (269, 129), (269, 130), (269, 131), (269, 132), (269, 133), (269, 134), (269, 135), (269, 136), (269, 137), (269, 138), (269, 139), (269, 140), (269, 141), (269, 142), (269, 143), (269, 144), (269, 145), (269, 146), (269, 147), (269, 148), (269, 149), (269, 150), (269, 151), (269, 152), (269, 153), (269, 154), (269, 155), (269, 156), (269, 157), (269, 158), (269, 159), (269, 160), (269, 161), (269, 162), (269, 163), (269, 164), (269, 165), (269, 166), (269, 167), (269, 168), (269, 169), (269, 170), (269, 171), (269, 172), (269, 173), (269, 174), (269, 175), (269, 176), (269, 177), (269, 178), (269, 179), (269, 180), (269, 181), (269, 182), (269, 183), (269, 184), (269, 185), (269, 186), (269, 187), (269, 188), (269, 189), (269, 190), (269, 191), (269, 192), (269, 193), (269, 194), (269, 195), (269, 196), (269, 197), (269, 198), (269, 199), (269, 200), (269, 201), (269, 202), (269, 203), (269, 204), (269, 205), (269, 206), (269, 207), (269, 208), (269, 209), (269, 210), (269, 211), (269, 212), (269, 213), (269, 214), (269, 215), (269, 216), (269, 217), (269, 218), (269, 219), (269, 220), (269, 221), (269, 222), (269, 223), (269, 224), (269, 225), (269, 226), (269, 227), (269, 228), (269, 229), (269, 230), (269, 231), (269, 232), (269, 233), (269, 234), (269, 235), (269, 236), (269, 237), (269, 238), (269, 239), (269, 240), (269, 241), (269, 242), (269, 243), (269, 244), (269, 245), (269, 246), (269, 247), (269, 248), (269, 249), (269, 250), (269, 251), (269, 252), (269, 253), (269, 254), (269, 255), (269, 256), (269, 257), (269, 258), (269, 259), (269, 260), (269, 261), (269, 262), (269, 263), (269, 264), (269, 265), (269, 266), (269, 267), (269, 268), (269, 269), (269, 270), (269, 271), (269, 272), (269, 273), (269, 274), (269, 275), (269, 276), (269, 277), (269, 278), (269, 279), (269, 280), (269, 281), (269, 282), (269, 283), (269, 284), (269, 285), (269, 286), (269, 287), (269, 288), (269, 289), (269, 290), (269, 291), (269, 292), (269, 293), (269, 294), (269, 295), (269, 296), (269, 297), (269, 298), (269, 299)]
    
    detection_cfg = yaml.safe_load(open('../conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('../conf/objects.yaml', 'r'))
    robot_cfg = objects_cfg['robot']
    rob = Robot(robot_cfg['radius'], robot_cfg['height'], robot_cfg['color'])
    rob.set_world_coordinates(robot_cfg['world_coordinates'])
    print(rob)

    tmp = regulated_move(rob)
    tmp.go(path)