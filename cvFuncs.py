# USAGE
# python match.py --template cod_logo.png --images images

# import the necessary packages
import numpy as np
import argparse
import imutils
import glob
import cv2
import time


def isolate(image, color):
    # blur and conver to convert to hsv color space
    # cv2.imshow("image",image);
    # define the list of boundaries with associated colors

    # blurred = cv2.GaussianBlur(image, (11, 11), 0)

    # black = 0 , white = 255
    if color == "black":
        mask = cv2.threshold(image, thresh=0, maxval=50,
                             type=cv2.THRESH_BINARY)
    elif color == "white":
        mask = cv2.threshold(image, thresh=180, maxval=255,
                             type=cv2.THRESH_BINARY)

    return mask
    # show the images


def adjustImage(img):
    brightness = 0
    contrast = 80
    img = np.int16(img)
    img = img * (contrast / 127 + 1) - contrast + brightness
    img = np.clip(img, 0, 255)
    img = np.uint8(img)
    return img


def findCircleCtrPt(gray, color, visBool=None):

    # Blur using 3 * 3 kernel.
    gray_blurred = cv2.blur(gray, (3, 3))

    # Apply Hough transform on the blurred image.
    detected_circles = cv2.HoughCircles(gray_blurred,
                                        cv2.HOUGH_GRADIENT, 1, 15, param1=50,
                                        param2=25, minRadius=12, maxRadius=16)


    # Draw circles that are detected.
    if detected_circles is not None and visBool:

        # Convert the circle parameters a, b and r to integers.
        detected_circles = np.uint16(np.around(detected_circles))

        for pt in detected_circles[0, :]:
            a, b, r = pt[0], pt[1], pt[2]

            # Draw the circumference of the circle.
            cv2.circle(gray, (a, b), r, (0, 255, 0), 2)

            # Draw a small circle (of radius 1) to show the center.
            cv2.circle(gray, (a, b), 1, (0, 0, 255), 3)
            #print(pt, end='')
            cv2.imshow("Detected Circle", gray)

    return detected_circles
