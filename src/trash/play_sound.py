#!/usr/bin/env python

#from robolab_turtlebot import Turtlebot, sleep

import sys
# setting path
sys.path.append('home/TUTLE_FRANKLIN/src/robolab_turtlebot')

from robolab_turtlebot import Turtlebot, sleep
from rospy import Rate

def main():
    turtle = Turtlebot()
    sleep(1)

    turtle.play_sound()
    sleep(0.3)
    turtle.play_sound(1)
    sleep(0.3)
    turtle.play_sound(2)
    sleep(0.3)
    turtle.play_sound(3)
    sleep(0.3)
    turtle.play_sound(4)
    sleep(0.3)
    turtle.play_sound(5)
    sleep(0.3)
    turtle.play_sound(6)
    sleep(1)


if __name__ == '__main__':
    main()
