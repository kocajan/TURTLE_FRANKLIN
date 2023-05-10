import os
import sys
import numpy as np
import imageio

# setting path
sys.path.append('home/TUTLE_FRANKLIN/src/robolab_turtlebot')

from robolab_turtlebot import Turtlebot, Rate

counter = 0
shoot = True


def capture_images():
    """Captures images from the camera and saves them to the given directory
    :param camera: Camera object
    :param directory: Directory to save the images to
    :param config: Configuration file
    """
    path = os.path.dirname(os.path.realpath(__file__))
    path = os.path.join(path, 'camera/shoot7')
    os.makedirs(path, exist_ok=True)

    turtle = Turtlebot()

    turtle.register_button_event_cb(button_cb)

    rate = Rate(1)
    while not turtle.is_shutting_down():
        rate.sleep()


def button_cb(event):
    global counter
    global shoot
    if shoot:
        # Create turtle object
        turtle = Turtlebot(rgb=True, pc=True)

        # Set path to save image and point cloud
        image_path = os.path.join('camera/shoot7', f'RGB{counter}.png')
        point_cloud_path = os.path.join('camera/shoot7', f'PC{counter}.npy')

        # Capture RGB image
        turtle.wait_for_rgb_image()
        bgr = turtle.get_rgb_image()

        # Convert BGR to RGB and save image
        rgb = bgr[...,::-1]
        imageio.imwrite(image_path, rgb)
        print(f"Saved image to {image_path}")

        # Capture point cloud
        turtle.wait_for_point_cloud()
        pc = turtle.get_point_cloud()

        # Save point cloud
        np.save(point_cloud_path, pc)
        print(f"Saved point cloud to {image_path}")

        # Compare sizes of RGB image and point cloud
        # print(f"RGB image shape: {rgb.shape}")
        # print(f"Point cloud shape: {pc.shape}")

        # Increment counter and set shoot to False
        counter += 1
        shoot = False

    else:
        shoot = True


if __name__ == '__main__':
    capture_images()
