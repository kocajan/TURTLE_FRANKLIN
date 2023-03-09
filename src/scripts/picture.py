import os
import sys
import numpy as np
#from scipy.misc import imsave
import imageio

# setting path
sys.path.append('home/TUTLE_FRANKLIN/src/robolab_turtlebot')

from robolab_turtlebot import Turtlebot, Rate

counter = 0


def button_cb(event):
    global counter
    turtle = Turtlebot(rgb=True)
    turtle.wait_for_rgb_image()
    rgb = turtle.get_rgb_image()
    image_path = os.path.join('camera', f'{counter}.png')
    #imsave(image_path, rgb)
    imageio.imwrite(image_path, rgb)
    print(f"Saved image to {image_path}")
    counter += 1


def capture_images():
    """Captures images from the camera and saves them to the given directory
    :param camera: Camera object
    :param directory: Directory to save the images to
    :param config: Configuration file
    """
    #os.makedirs(directory, exist_ok=True)

    turtle = Turtlebot()

    turtle.register_button_event_cb(button_cb)

    rate = Rate(1)
    while not turtle.is_shutting_down():
        rate.sleep()

if __name__ == '__main__':
    capture_images()
