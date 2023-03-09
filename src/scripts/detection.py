import numpy as np
import cv2 as cv
import yaml


def get_parking_spot_orientation(img: np.ndarray, piles: np.ndarray, detection_cfg: dict) -> None:
    """
    Find the orientation of the parking spot.
    :param img: The image to process.
    :param piles: Contours of the magenta piles of the parking spot
    :param detection_cfg: The detection config.
    :return: None
    """
    # find a bottom point of the piles
    bottom_point = None
    for pile_contour in piles:
        extBot = tuple(pile_contour[pile_contour[:, :, 1].argmax()][0])
        cv.circle(img, extBot, 8, (255, 255, 0), -1)
        # TODO


def segmentation(img: np.ndarray, config: dict):
    """
    Find objects of interest in the image. Distinguish between objects of interest in the foreground and background.
    """
    # Convert to HSV
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    for color in config['colors'].values():
        # Threshold the HSV image to get only the selected color
        lower = np.array(color['lower'])
        upper = np.array(color['upper'])
        mask = cv.inRange(hsv, lower, upper)
        output = cv.bitwise_and(img, img, mask=mask)

        # # Get rid of noise
        # kernel = np.ones((5, 5), np.uint8)
        # output = cv.morphologyEx(output, cv.MORPH_OPEN, kernel)

        # Make the image binary
        output = cv.cvtColor(output, cv.COLOR_BGR2GRAY)

        # Find contours
        contours, hierarchy = cv.findContours(output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Draw contours
        copy_img = img.copy()
        cv.drawContours(copy_img, contours, -1, (0, 255, 0), 1)

        if color['color'] == 'red' or color['color'] == 'green' or color['color'] == 'blue':
            # Obstacles
            cv.imshow(color['color'], copy_img)
        else if color['color'] == 'magenta':
            # Entrance
        else:
            # Garage

        # Get the smallest rectangle that contains the contour
        for cnt in contours:
            if cv.contourArea(cnt) < 300 or color['color'] == 'yellow':
                continue
            rect = cv.minAreaRect(cnt)
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(copy_img, [box], 0, (0, 0, 255), 1)

        if color['color'] == 'magenta':
            get_parking_spot_orientation(img, contours, config)

        # Show the result
        output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
        final = np.concatenate((copy_img, output), axis=1)
        cv.imshow(color['color'], final)
        cv.waitKey(0)


def process_image(img: np.ndarray) -> None:
    """
    Process the image and show the result.
    :param img: The image to process.
    :return: None
    """
    # Load config
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))

    # Find objects of interest
    segmentation(img, detection_cfg)


if __name__ == '__main__':
    for i in range(9):
        if i == False:
            continue
        img = cv.imread(f'camera/shoot2/{i}.png')
        process_image(img)
        cv.waitKey(0)
        cv.destroyAllWindows()
