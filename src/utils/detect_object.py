"""
Provides functions to perform the actual object detection.
Utilised by vision.py Node 
"""

import cv2
import numpy as np
import math


# Colour Thresholding Variables Hue - [0,179]; Saturation - [0,255]; Value - [0,255]
COLORS = {
    'block-mask': ([0, 0, 194], [179, 255, 255]),
    'icon-mask': ([0, 102, 0], [179, 255, 255])
}

CIRCULARITY_BOUNDS = [0.85, 0.65]

BLOCK_AREA_MIN = 1500
BLOCK_AREA_MAX = 10000

IM_WIDTH = 640
IM_HEIGHT = 480

CONVEYOR_WIDTH = 0.45 # 0.5
HEIGHT_OF_CONVEYOR = 0.25
HEIGHT_OF_ROBOT = 0.3
Y_DIST_FROM_ROBOT = 0.5

DIST_PER_PIXEL = CONVEYOR_WIDTH / IM_HEIGHT
WIDTH_DIST = IM_WIDTH * DIST_PER_PIXEL
HEIGHT_DIST = IM_HEIGHT * DIST_PER_PIXEL

def colorThreshold(im_rgb, name):
    """
    Performs HSV colour thresholding based on globally defined ranges
    Inputs
        im_rgb: <np.array HxWx3> Image to threshold
        name: <string> Colour range to use {pink-corner, internal-wall, robot}

    Outputs
        mask: <np.array HxW> Boolean Mask
    """
    # Get Lower and Upper Range for specified colour
    lower = np.array(COLORS[name][0])
    upper = np.array(COLORS[name][1])

    # Convert to HSV and perform thresholding
    im_hsv = cv2.cvtColor(im_rgb, cv2.COLOR_RGB2HSV)
    mask = cv2.inRange(im_hsv, lower, upper)

    return mask


def blockMask(im_rgb):
    """
    MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
    getBlockMask function isolates the block/s present in image file 'image'
    Written by Jason Jia Sheng Quek | z5117285
    Modified by Rowena Dai | z5075936 - returns binary mask only
    """
    block_mask = colorThreshold(im_rgb, 'block-mask')


    # Imfill https://www.learnopencv.com/filling-holes-in-an-image-using-opencv-python-c/
    im_floodfill = block_mask.copy()
    h, w = block_mask.shape[:2]

    mask = np.zeros((h+2, w+2), np.uint8)
    cv2.floodFill(im_floodfill, mask, (0,0), 255);
    im_floodfill_inv = cv2.bitwise_not(im_floodfill)
    
    # Combine the two images to get the foreground.
    im_fill = block_mask | im_floodfill_inv

    return im_fill


def getColor(im_rgb, x, y):
    """
    Gets red|green|blue based off the rgb value of the x,y pixel in im_rgb
    Written by Rowena Dai | z5075936
    """
    max_index = np.argmax(im_rgb[y,x,:])

    if max_index == 0: return 'red'
    elif max_index == 1: return 'green'
    elif max_index == 2: return 'blue'


def getShape(im_block_rgb):
    """
    MTRN4230 T2 2020 - Group Assignment: Computer Vision & Image Processing
    Given a cropped image of a single block surface, getShape function
    returns the shape of the icon (circle/square/triangle)
    Written by Jason Jia Sheng Quek | z5117285
    """

    # HSV colour thresholding produces a binary image 'icon_mask'
    icon_mask = colorThreshold(im_block_rgb, 'icon-mask')
    
    # Extract detected icon's circularity
    _, contours, _ = cv2.findContours(icon_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_contour = max(contours, key=lambda cnt: cv2.contourArea(cnt))

    area = cv2.contourArea(max_contour)
    perimeter = cv2.arcLength(max_contour,True)
    circ = 4*math.pi*area / perimeter**2

    # Identify and return icon shape based on 'circ' range
    if circ >= CIRCULARITY_BOUNDS[0]: shape = 'circle'
    elif circ <= CIRCULARITY_BOUNDS[1]: shape = 'triangle'
    else: shape = 'square'

    return shape, circ


def transformCoordinates(x, y):
    """
    Transforms x,y pixel coordinates to the actual x,y,z coordinates in the Gazebo environment
    """
    x_t = HEIGHT_DIST/2 - y/IM_HEIGHT*HEIGHT_DIST + Y_DIST_FROM_ROBOT
    y_t = -x/IM_WIDTH*WIDTH_DIST + WIDTH_DIST/2
    z_t = HEIGHT_OF_CONVEYOR - HEIGHT_OF_ROBOT

    x_t = -x_t
    y_t = -y_t

    return x_t, y_t, z_t


def detectObject(im_rgb):
    """
    Perform object detection for the given image input
    Returns two outputs
    result - Result of the object detection (color, shape, location)
    im_debug - Visual feedback shown in the Web GUI
    """
    block_mask = blockMask(im_rgb)

    num_cc, labels, stats, centroids = cv2.connectedComponentsWithStats(block_mask, connectivity=8)
    components_sorted = sorted(zip(stats, centroids), key=lambda x: x[0][4], reverse=True) # Sort by component area descending

    for i in range(num_cc):
        curr_stat, curr_centroid = components_sorted[i]
        left, top, width, height, area = curr_stat
        
        if not (BLOCK_AREA_MIN <= area <= BLOCK_AREA_MAX): continue

        # Centroid
        x = round(curr_centroid[0])
        y = round(curr_centroid[1])

        # Get Color
        color = getColor(im_rgb, x, y)

        # Get Shape
        # Crop
        im_block_rgb = im_rgb[top:top+height, left:left+width, :]
        shape, circ = getShape(im_block_rgb)

        x_t, y_t, z_t = transformCoordinates(x, y)

        # Debugging Image
        im_debug = im_rgb.copy()

        # Bounding Rectangle
        tl = (left,top)
        br = (left+width, top+height)
        cv2.rectangle(im_debug, tl, br, (0,0,255), 5)

        # Text Description
        text_object_pos = (int(x - width/2), int(y - height))
        text_object = '{} {}'.format(color, shape)
        cv2.putText(im_debug, text_object, text_object_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(218,57,226), thickness=2)

        text_loc_pos = (int(x - width/2), int(y + height))
        text_loc = 'x={}, y={}'.format(round(x_t,2), round(y_t,2))
        cv2.putText(im_debug, text_loc, text_loc_pos, fontFace=cv2.FONT_HERSHEY_SIMPLEX, fontScale=0.75, color=(246,251,87), thickness=2)

        # Add Marker
        cv2.drawMarker(im_debug, (int(x), int(y)), color=(0, 0, 255), markerType=cv2.MARKER_TILTED_CROSS, markerSize=24, thickness=3)

        im_debug = cv2.cvtColor(im_debug, cv2.COLOR_RGB2BGR)

        result = (color, shape, x_t, y_t, z_t)
        return (result, im_debug)

