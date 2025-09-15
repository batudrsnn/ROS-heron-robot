#! /usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def callback_hmin(value):
    global h_min
    h_min = value

def callback_hmax(value):
    global h_max
    h_max = value

def callback_smin(value):
    global s_min
    s_min = value

def callback_smax(value):
    global s_max
    s_max = value

def callback_vmin(value):
    global v_min
    v_min = value

def callback_vmax(value):
    global v_max
    v_max = value

def make_bars():
    cv2.namedWindow('HSV', 0)
    cv2.createTrackbar('hmin', 'HSV', 0, 255, callback_hmin)
    cv2.createTrackbar('hmax', 'HSV', 255, 255, callback_hmax)
    cv2.createTrackbar('smin', 'HSV', 0, 255, callback_smin)
    cv2.createTrackbar('smax', 'HSV', 255, 255, callback_smax)
    cv2.createTrackbar('vmin', 'HSV', 0, 255, callback_vmin)
    cv2.createTrackbar('vmax', 'HSV', 255, 255, callback_vmax)

def image_callback(data):
    global bridge
    global h_min
    global h_max
    global s_min
    global s_max
    global v_min
    global v_max
    try:
        cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    except CvBridgeError as e:
        print(e)

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, (h_min, s_min, v_min), (h_max, s_max, v_max))

    cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)

    cv2.imshow('HSV', cv_image)
    cv2.waitKey(3)

def main():
    global bridge
    global h_min
    global h_max
    global s_min
    global s_max
    global v_min
    global v_max
    h_min, s_min, v_min = 0, 0, 0
    h_max, s_max, v_max = 255, 255, 255
    make_bars()
    cv2.imshow('HSV', np.zeros((480, 840, 3), dtype=np.uint8))
    cv2.waitKey(3)
    rospy.init_node('hsv_sampler', anonymous=True)
    bridge = CvBridge()
    rospy.Subscriber('/realsense/rgb/image_raw', Image, callback=image_callback)
    rospy.spin()


if __name__ == '__main__':
    main()
