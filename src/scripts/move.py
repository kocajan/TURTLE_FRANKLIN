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

    def execute_move(self, moves_list):
        for single_mv in moves_list:
            print("dummy")
        pass

    #exports nonlinear path, to linear
    #this is unneccessary because of A* logic
    def non_to_linear_path(self, non_linear_path):
        linear_path = list()
        return linear_path

    #format of neighbours is UP, DOWN, DIAGONAL_UP_RIGHT, DIAGONAL_UP_LEFT, DIAGONAL_DOWN_RIGHT, DIAGONAL_DOWN_LEFT, RIGHT_ANGLE_LEFT, RIGHT_ANGLE_RIGHT
    #zaroven se nemuzu divat na predchozi pozici, asi to chce do fce jako argument
    def generate_neighbours(self, current_pos, prev_pos):
        neighbours = list()
        neighbours.append( (current_pos[0],current_pos[1] + 1) )    # up
        if (prev_pos != None):
            neighbours.append( (current_pos[0], current_pos[1] - 1) )   # down

        neighbours.append( (current_pos[0] + 1, current_pos[1] + 1) ) # diagonal up right
        neighbours.append( (current_pos[0] -1, current_pos[1] + 1) )   #diagonal up left
        if (prev_pos != None):
            neighbours.append( (current_pos[0] + 1, current_pos[1] - 1) )   #diagonal down right
            neighbours.append( (current_pos[0] -1, current_pos[1] - 1) )   #diagonal down left

        neighbours.append( (current_pos[0] -1, current_pos[1]) )   #right angle left
        neighbours.append( (current_pos[0] + 1, current_pos[1]) )   #right angle right

        for index, neighbour in enumerate(neighbours):
            if(neighbour == prev_pos):
                neighbours[index].pop() # removing previous position, it is invalid neighbour
        return neighbours

    def check_trend(self, index, prev_index, rotation, translation):
        if(index != prev_index and prev_index != None):
            return single_mv(rotation, translation)
        return None
    #creates single_mv classes. single_mv classes are then executed
    def move_sequence(self):
        rotation = 0
        translation = 0
        sequence = list()
        prev_coord = None
        prev_index = None
        #TODO vyresit posledni node, ta se jenom apenduje, bude stejna jako vsechny ostatni

        for pos in range(len(self.move_coords)):
            next_neighbours = self.generate_neighbours(self.move_coords[pos], prev_coord)
            for index, neigh in enumerate(next_neighbours):
                if(neigh == self.move_coords[pos+1]):
                    if(index == 0):     #up
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        rotation = 0
                        translation = translation + 1
                    elif(index == 1):   #down
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        rotation = 0
                        translation = translation - 1
                    elif(index == 2):   #diagonal up right
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 45
                        translation = translation + 1
                    elif(index == 3):   #diagonal up left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -45
                        translation = translation + 1
                    elif(index == 4):   #diagonal down right
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -135
                        translation = translation - 1
                    elif(index == 5):   #diagonal down left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 135
                        translation = translation - 1
                    elif(index == 6):   #90 deg left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -90
                        translation = 0
                    elif(index == 7):   #90 deg left
                        if((tmp := self.check_trend(index, prev_index, rotation, translation)) != None):
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 90
                        translation = 0
                    prev_index = index

            prev_coord = self.move_coords[pos]
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
        

path = [(0, 250), (1, 251), (2, 252), (3, 253), (4, 254), (5, 255), (6, 256), (7, 257), (8, 258), (9, 259), (10, 260), (11, 261), (12, 262), (13, 263), (14, 264), (15, 265), (16, 266), (17, 267), (18, 268), (19, 269), (20, 270), (21, 271), (22, 272), (23, 273), (24, 274), (25, 275), (26, 276), (27, 277), (28, 278), (29, 279), (30, 280), (31, 281), (32, 282), (33, 283), (34, 284), (35, 285), (36, 286), (37, 287), (38, 288), (39, 289), (40, 290), (41, 291), (42, 292), (43, 293), (44, 294), (45, 295), (46, 296), (47, 297), (48, 298), (49, 299), (49, 300), (49, 301), (49, 302), (49, 303), (49, 304), (49, 305), (49, 306), (49, 307), (49, 308), (49, 309), (49, 310), (49, 311), (49, 312), (49, 313), (49, 314), (49, 315), (49, 316), (49, 317), (49, 318), (49, 319), (49, 320), (49, 321), (49, 322), (49, 323), (49, 324), (49, 325), (49, 326), (49, 327), (49, 328), (49, 329), (49, 330), (49, 331), (49, 332), (49, 333), (49, 334), (49, 335), (49, 336), (49, 337), (49, 338), (49, 339), (49, 340), (49, 341), (49, 342), (49, 343), (49, 344), (49, 345), (49, 346), (49, 347), (49, 348), (49, 349), (49, 350), (49, 351), (49, 352), (49, 353), (49, 354), (49, 355), (49, 356), (49, 357), (49, 358), (49, 359), (49, 360), (49, 361), (49, 362), (49, 363), (49, 364), (49, 365), (49, 366), (49, 367), (49, 368), (49, 369), (49, 370), (49, 371), (49, 372), (49, 373), (49, 374), (49, 375), (49, 376), (49, 377), (49, 378), (49, 379), (49, 380), (49, 381), (49, 382), (49, 383), (49, 384), (49, 385), (49, 386), (49, 387), (49, 388), (49, 389), (49, 390), (49, 391), (49, 392), (49, 393), (49, 394), (49, 395), (49, 396), (49, 397), (49, 398), (49, 399), (49, 400), (50, 401), (51, 401), (52, 401), (53, 401), (54, 401), (55, 401), (56, 401), (57, 401), (58, 401), (59, 401), (60, 401), (61, 401), (62, 401), (63, 401), (64, 401), (65, 401), (66, 401), (67, 401), (68, 401), (69, 401), (70, 401), (71, 401), (72, 401), (73, 401), (74, 401), (75, 401), (76, 401), (77, 401), (78, 401), (79, 401), (80, 401), (81, 401), (82, 401), (83, 401), (84, 401), (85, 401), (86, 401), (87, 401), (88, 401), (89, 401), (90, 401), (91, 401), (92, 401), (93, 401), (94, 401), (95, 401), (96, 401), (97, 401), (98, 401), (99, 401), (100, 401), (101, 400), (102, 399), (103, 398), (104, 397), (105, 396), (106, 395), (107, 394), (108, 393), (109, 392), (110, 391), (111, 390), (112, 389), (113, 388), (114, 387), (115, 386), (116, 385), (117, 384), (118, 383), (119, 382), (120, 381), (121, 380), (122, 379), (123, 378), (124, 377), (125, 376), (126, 375), (127, 374), (128, 373), (129, 372), (130, 371), (131, 370), (132, 369), (133, 368), (134, 367), (135, 366), (136, 365), (137, 364), (138, 363), (139, 362), (140, 361), (141, 360), (142, 359), (143, 358), (144, 357), (145, 356), (146, 355), (147, 354), (148, 353), (149, 352), (150, 351), (151, 350), (152, 349), (153, 348), (154, 347), (155, 346), (156, 345), (157, 344), (158, 343), (159, 342), (160, 341), (161, 340), (162, 339), (163, 338), (164, 337), (165, 336), (166, 335), (167, 334), (168, 333), (169, 332), (170, 331), (171, 330), (172, 329), (173, 328), (174, 327), (175, 326), (176, 325), (177, 324), (178, 323), (179, 322), (180, 321), (181, 320), (182, 319), (183, 318), (184, 317), (185, 316), (186, 315), (187, 314), (188, 313), (189, 312), (190, 311), (191, 310), (192, 309), (193, 308), (194, 307), (195, 306), (196, 305), (197, 304), (198, 303), (199, 302), (199, 301), (199, 300), (199, 299), (199, 298), (199, 297), (199, 296), (199, 295), (199, 294), (199, 293), (199, 292), (199, 291), (199, 290), (199, 289), (199, 288), (199, 287), (199, 286), (199, 285), (199, 284), (199, 283), (199, 282), (199, 281), (199, 280), (199, 279), (199, 278), (199, 277), (199, 276), (199, 275), (199, 274), (199, 273), (199, 272), (199, 271), (199, 270), (199, 269), (199, 268), (199, 267), (199, 266), (199, 265), (199, 264), (199, 263), (199, 262), (199, 261), (199, 260), (199, 259), (199, 258), (199, 257), (199, 256), (199, 255), (199, 254), (199, 253), (199, 252), (199, 251), (199, 250), (199, 249), (199, 248), (199, 247), (199, 246), (199, 245), (199, 244), (199, 243), (199, 242), (199, 241), (199, 240), (199, 239), (199, 238), (199, 237), (199, 236), (199, 235), (199, 234), (199, 233), (199, 232), (199, 231), (199, 230), (199, 229), (199, 228), (199, 227), (199, 226), (199, 225), (199, 224), (199, 223), (199, 222), (199, 221), (199, 220), (199, 219), (199, 218), (199, 217), (199, 216), (199, 215), (199, 214), (199, 213), (199, 212), (199, 211), (199, 210), (199, 209), (199, 208), (199, 207), (199, 206), (199, 205), (199, 204), (199, 203), (199, 202), (199, 201), (199, 200), (199, 199), (199, 198), (199, 197), (199, 196), (199, 195), (199, 194), (199, 193), (199, 192), (199, 191), (199, 190), (199, 189), (199, 188), (199, 187), (199, 186), (199, 185), (199, 184), (199, 183), (199, 182), (199, 181), (199, 180), (199, 179), (199, 178), (199, 177), (199, 176), (199, 175), (199, 174), (199, 173), (199, 172), (199, 171), (199, 170), (199, 169), (199, 168), (199, 167), (199, 166), (199, 165), (199, 164), (199, 163), (199, 162), (199, 161), (199, 160), (199, 159), (199, 158), (199, 157), (199, 156), (199, 155), (199, 154), (199, 153), (199, 152), (199, 151), (199, 150), (199, 149), (199, 148), (199, 147), (199, 146), (199, 145), (199, 144), (199, 143), (199, 142), (199, 141), (199, 140), (199, 139), (199, 138), (199, 137), (199, 136), (199, 135), (199, 134), (199, 133), (199, 132), (199, 131), (199, 130), (199, 129), (199, 128), (199, 127), (199, 126), (199, 125), (199, 124), (199, 123), (199, 122), (199, 121), (199, 120), (199, 119), (199, 118), (199, 117), (199, 116), (199, 115), (199, 114), (199, 113), (199, 112), (199, 111), (199, 110), (199, 109), (199, 108), (199, 107), (199, 106), (199, 105), (199, 104), (199, 103), (199, 102), (199, 101), (199, 100), (200, 99), (201, 99), (202, 99), (203, 99), (204, 99), (205, 99), (206, 99), (207, 99), (208, 99), (209, 99), (210, 99), (211, 99), (212, 99), (213, 99), (214, 99), (215, 99), (216, 99), (217, 99), (218, 99), (219, 99), (220, 99), (221, 99), (222, 99), (223, 99), (224, 99), (225, 99), (226, 99), (227, 99), (228, 99), (229, 99), (230, 99), (231, 99), (232, 99), (233, 99), (234, 99), (235, 99), (236, 99), (237, 99), (238, 99), (239, 99), (240, 99), (241, 99), (242, 99), (243, 99), (244, 99), (245, 99), (246, 99), (247, 99), (248, 99), (249, 99), (250, 99), (251, 100), (252, 101), (253, 102), (254, 103), (255, 104), (256, 105), (257, 106), (258, 107), (259, 108), (260, 109), (261, 110), (262, 111), (263, 112), (264, 113), (265, 114), (266, 115), (267, 116), (268, 117), (269, 118), (270, 119), (271, 120), (272, 121), (273, 122), (274, 123), (275, 124), (276, 125), (277, 126), (278, 127), (279, 128), (280, 129), (280, 130), (280, 131), (280, 132), (280, 133), (280, 134), (280, 135), (280, 136), (280, 137), (280, 138), (280, 139), (280, 140), (280, 141), (280, 142), (280, 143), (280, 144), (280, 145), (280, 146), (280, 147), (280, 148), (280, 149), (280, 150), (280, 151), (280, 152), (280, 153), (280, 154), (280, 155), (280, 156), (280, 157), (280, 158), (280, 159), (280, 160), (280, 161), (280, 162), (280, 163), (280, 164), (280, 165), (280, 166), (280, 167), (280, 168), (280, 169), (280, 170), (280, 171), (280, 172), (280, 173), (280, 174), (280, 175), (280, 176), (280, 177), (280, 178), (280, 179), (280, 180), (280, 181), (280, 182), (280, 183), (280, 184), (280, 185), (280, 186), (280, 187), (280, 188), (280, 189), (280, 190), (280, 191), (280, 192), (280, 193), (280, 194), (280, 195), (280, 196), (280, 197), (280, 198), (280, 199), (280, 200), (280, 201), (280, 202), (280, 203), (280, 204), (280, 205), (280, 206), (280, 207), (280, 208), (280, 209), (280, 210), (280, 211), (280, 212), (280, 213), (280, 214), (280, 215), (280, 216), (280, 217), (280, 218), (280, 219), (280, 220), (280, 221), (280, 222), (280, 223), (280, 224), (280, 225), (280, 226), (280, 227), (280, 228), (280, 229), (280, 230), (280, 231), (280, 232), (280, 233), (280, 234), (280, 235), (280, 236), (280, 237), (280, 238), (280, 239), (280, 240), (280, 241), (280, 242), (280, 243), (280, 244), (280, 245), (280, 246), (280, 247), (280, 248), (280, 249), (280, 250), (280, 251), (280, 252), (280, 253), (280, 254), (280, 255), (280, 256), (280, 257), (280, 258), (280, 259), (280, 260), (280, 261), (280, 262), (280, 263), (280, 264), (280, 265), (280, 266), (280, 267), (280, 268), (280, 269), (280, 270), (280, 271), (280, 272), (280, 273), (280, 274), (280, 275), (280, 276), (280, 277), (280, 278), (280, 279), (280, 280)]

tmp = move(None)
#tmp.rotate_degrees(-60)
tmp.go_straight(100,1)
#tmp.rotate_degrees(-180)
