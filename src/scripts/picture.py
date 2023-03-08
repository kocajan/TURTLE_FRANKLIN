import os
import numpy as np
import cv2 as cv
from robolab_turtlebot import Turtlebot, Rate

counter = 0


def button_cb(event):
    turtle = Turtlebot(rgb=True)
    turtle.wait_for_rgb_image()
    image = turtle.get_rgb_image()
    cv_image = np.array(image.getData(), dtype="uint8").reshape((image.getRows(), image.getCols(), 3))
    image_path = os.path.join('camera', f'{counter}.png')
    cv.imwrite(image_path, cv_image)


def capture_images(camera, directory: str, config: dict):
    """Captures images from the camera and saves them to the given directory
    :param camera: Camera object
    :param directory: Directory to save the images to
    :param config: Configuration file
    """
    os.makedirs(directory, exist_ok=True)

    turtle = Turtlebot()

    turtle.register_button_event_cb(button_cb)

    rate = Rate(1)
    while not turtle.is_shutting_down():
        rate.sleep()
