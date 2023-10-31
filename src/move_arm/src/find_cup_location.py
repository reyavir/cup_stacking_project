import cv2 as cv
import numpy as np

test_img = 'test/cups.jpg'
pickup_length = 32.5 
pyramid_length = 26.5
pickup_width = 17.5
# robot coordinates --> hardcode here

# for color filtering (stretch)
# Threshold of blue in HSV space 
# lower_blue = np.array([60, 35, 140]) 
# upper_blue = np.array([180, 255, 255]) 

# find ar tag marker corners
# need alvar ar tag tracking package
# visualization marker for it?

# 
def get_cup_locations(ar_tag_locations, im):
    cups = []
    # need 4 point correspondences --> feed in ar tag markers --> square coordinates in pixel dimensions
    output = [[0,0], [], [], []] # define based on relative ar tag locations - ORDER MATTERS HERE
    maxWidth = None
    maxHeight = None # define based on relative length of table region
    H = cv.getPerspectiveTransform(ar_tag_locations, output) 
    rectified_img = cv.warpPerspective(im, H, (maxWidth, maxHeight), flags=cv.INTER_LINEAR)

    img = rectified_img # find circles based on straightened out image
    # cap = cv.VideoCapture(0)

    img = cv.medianBlur(img,5)
    circles = cv.HoughCircles(img,cv.HOUGH_GRADIENT,1,20,
                                param1=50,param2=30,minRadius=0,maxRadius=0)
    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        center_x = i[0]
        center_y = i[1]
        # note that this is in pixels
        # color filtering would go here
        cups.append([center_x, center_y])
    return cups

