import numpy as np
import math
import yaml
from single_mv import single_mv
from robolab_turtlebot import Turtlebot, Rate, get_time

class move:
    move_coords = list(list())  #2D array of moving coords

    def __init__(self, coords):
        self.detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
        self.move_coords = coords
        self.turtle = Turtlebot()
        self.rate = Rate(10) # co dela tahle funkce?

    def execute_move(self):
        moves_to_execute = self.move_sequence()
        for index, move in enumerate(moves_to_execute):
            print(move.get_go_distance(), move.get_rotation())
            self.rotate_degrees(move.get_rotation())
            self.go_straight(move.get_go_distance(), np.sign(move.get_go_distance()))


    #format of neighbours is UP, DOWN, DIAGONAL_UP_RIGHT, DIAGONAL_UP_LEFT, DIAGONAL_DOWN_RIGHT, DIAGONAL_DOWN_LEFT, RIGHT_ANGLE_LEFT, RIGHT_ANGLE_RIGHT
    #zaroven se nemuzu divat na predchozi pozici, asi to chce do fce jako argument
    def generate_neighbours(self, current_pos, prev_pos):
        neighbours = list()
        neighbours.append( (current_pos[0],current_pos[1] + 1) )    # up
        if (prev_pos != None):
            neighbours.append( (current_pos[0], current_pos[1] - 1) )   # down
        elif (prev_pos == None):
            neighbours.append(None)

        neighbours.append( (current_pos[0] + 1, current_pos[1] + 1) ) # diagonal up right
        neighbours.append( (current_pos[0] -1, current_pos[1] + 1) )   #diagonal up left
        if (prev_pos != None):
            neighbours.append( (current_pos[0] + 1, current_pos[1] - 1) )   #diagonal down right
            neighbours.append( (current_pos[0] -1, current_pos[1] - 1) )   #diagonal down left
        elif (prev_pos == None):
            neighbours.append(None)
            neighbours.append(None)

        neighbours.append( (current_pos[0] -1, current_pos[1]) )   #right angle left
        neighbours.append( (current_pos[0] + 1, current_pos[1]) )   #right angle right

        for index, neighbour in enumerate(neighbours):
            if(neighbour == prev_pos and prev_pos != None):
                neighbours.remove(neighbour) # removing previous position, it is invalid neighbour
        return neighbours

    def check_trend(self, index, prev_index, rotation, translation):
        if(index != prev_index and prev_index != None):
            tmp = single_mv(rotation, translation)
            return tmp
        return None
    #creates single_mv classes. single_mv classes are then executed
    def move_sequence(self):
        rotation = 0
        translation = 0
        sequence = list()
        prev_coord = None
        prev_index = None
        prev_rotation = 0
        #TODO vyresit posledni node, ta se jenom apenduje, bude stejna jako vsechny ostatni

        for pos in range(len(self.move_coords)):
            next_neighbours = self.generate_neighbours(self.move_coords[pos], prev_coord)
            for index, neigh in enumerate(next_neighbours):
                if( (pos+1) < (len(self.move_coords)) and neigh == self.move_coords[pos+1] and neigh != None):
                    if(index == 0):     #up
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        rotation = - prev_rotation
                        translation = translation + 1
                        #break
                    elif(index == 1):   #down
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        rotation = - prev_rotation
                        translation = translation - 1
                        #break
                    elif(index == 2):   #diagonal up right
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 45 - prev_rotation
                        translation = translation + 1
                        #break
                    elif(index == 3):   #diagonal up left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -45 - prev_rotation
                        translation = translation + 1
                        #break
                    elif(index == 4):   #diagonal down right
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -135 - prev_rotation
                        translation = translation - 1
                        #break
                    elif(index == 5):   #diagonal down left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 135 - prev_rotation
                        translation = translation - 1
                        #break
                    elif(index == 6):   #90 deg left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -90 - prev_rotation
                        translation = 0
                        #break
                    elif(index == 7):   #90 deg left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 90 - prev_rotation
                        translation = 0
                        #break
                    prev_index = index
                    prev_rotation = abs(rotation)
                    break
            prev_coord = self.move_coords[pos]
        sequence.append(self.check_trend(index, 10, rotation, translation))
        print('here ', sequence[1].get_go_distance())
        return sequence
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
        return None #list(odometry_x_y[0], odometry_x_y[1])

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
        # TODO move compensation constant to yaml

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


if __name__ == '__main__':
    path =  [(250, 0), (251, 1), (252, 2), (253, 3), (253, 4), (253, 5), (253, 6), (253, 7), (253, 8), (253, 9), (253, 10), (253, 11), (253, 12), (253, 13), (253, 14), (253, 15), (253, 16), (253, 17), (253, 18), (253, 19), (253, 20), (253, 21), (253, 22), (253, 23), (253, 24), (253, 25), (253, 26), (253, 27), (253, 28), (253, 29), (253, 30), (253, 31), (253, 32), (253, 33), (253, 34), (253, 35), (253, 36), (253, 37), (253, 38), (253, 39), (253, 40), (253, 41), (253, 42), (253, 43), (253, 44), (253, 45), (253, 46), (253, 47), (253, 48), (253, 49), (253, 50), (253, 51), (253, 52), (253, 53), (253, 54), (253, 55), (253, 56), (253, 57), (253, 58), (253, 59), (253, 60), (253, 61), (253, 62), (253, 63), (253, 64), (253, 65), (253, 66), (253, 67)]

    tmp = move(path)
    #tmp.move_sequence()
    tmp.execute_move()
