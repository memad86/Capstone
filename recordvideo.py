#!/usr/bin/env python3
import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')
import sys
from sensor_msgs.msg import Image 
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
import rospy
import numpy as np
import traceback
import time, os
import argparse
from datetime import datetime

from cv_bridge import CvBridge, CvBridgeError
import cv2
bridge = CvBridge()

frame_size = (640,480)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
dir_path                  = os.path.expanduser('~/catkin_ws/src/drone_motion/videos/')
name = dir_path + 'output' + datetime.now().strftime("%d_%m_%Y_%H_%M_%S") + '.mp4'
print(name)
out = cv2.VideoWriter(name, fourcc, 20.0, frame_size)
show_track_bar = True

if show_track_bar:
    def empty_function_for_tracker(dummyinput):
        pass

    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 640, 240)
    cv2.createTrackbar("HUE Min", "HSV", 0, 179, empty_function_for_tracker)
    cv2.createTrackbar("HUE Max", "HSV", 179, 179, empty_function_for_tracker)
    cv2.createTrackbar("Sat Min", "HSV", 0, 255, empty_function_for_tracker)
    cv2.createTrackbar("Sat Max", "HSV", 255, 255, empty_function_for_tracker)
    cv2.createTrackbar("Value Min", "HSV", 0, 255, empty_function_for_tracker)
    cv2.createTrackbar("Value Max", "HSV", 255, 255, empty_function_for_tracker)

class ROS_Image:

    def __init__(self):
        rospy.Subscriber("/iris/cam_ventral/image_raw", Image, self.callback)
        self.cv_image = 0
        self.color_dict = {'YELLOW':{'min': np.array([19, 107, 0]), 'max': np.array([34, 255, 255])},
                           'RED':{'min': np.array([0, 166, 0]), 'max': np.array([10, 255, 255])}}
        self.lower_default = np.array([0,0,0])
        self.upper_default = np.array([179,255,255])
        self.show_contour = False

    def callback(self, data):
        try:
            self.cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
            self.cv_image_corrosion = bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)
    
    def save_data(self):
        try:
            out.write(cv2.resize(self.cv_image, frame_size))
        except:
            rospy.info("cannot save")

    def opencv_trackbar(self):
        h_min, h_max, s_min, s_max, v_min, v_max = 0,0,0,0,0,0
        if show_track_bar:
            h_min = cv2.getTrackbarPos('HUE Min', "HSV")
            h_max = cv2.getTrackbarPos('HUE Max', "HSV")
            s_min = cv2.getTrackbarPos('Sat Min', "HSV")
            s_max = cv2.getTrackbarPos('Sat Max', "HSV")
            v_min = cv2.getTrackbarPos('Value Min', "HSV")
            v_max = cv2.getTrackbarPos('Value Max', "HSV")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        if np.all(lower == self.lower_default) and np.all(upper == self.upper_default):
            lower = self.color_dict['RED']['min']
            upper = self.color_dict['RED']['max']
            lower_corrosion = self.color_dict['YELLOW']['min']
            upper_corrosion = self.color_dict['YELLOW']['max']
        else:
            pass
        self.imgHsv = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.mask = cv2.inRange(self.imgHsv, lower, upper)
        self.resulted_image = cv2.bitwise_and(self.cv_image, self.cv_image, mask = self.mask)

        self.imgHsv_corrosion = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2HSV)
        self.mask_corrosion = cv2.inRange(self.imgHsv_corrosion, lower_corrosion, upper_corrosion)

        # Creating contour to track red color
        contours, hierarchy = cv2.findContours(self.mask_corrosion,   cv2.RETR_TREE,      cv2.CHAIN_APPROX_SIMPLE)

        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            self.initial_time = time.time()
            if(area > 100):
                x, y, w, h = cv2.boundingRect(contour)
                self.cv_image_corrosion = cv2.rectangle(self.cv_image_corrosion, (x, y), (x + w, y + h), (107, 179, 179), 5)

if __name__ == '__main__':
    rospy.init_node("Color_detector", anonymous=False)
    rate = rospy.Rate(10)
    record_video = ROS_Image()
    while not rospy.is_shutdown():
        try:
            try:
                record_video.save_data()
                record_video.opencv_trackbar()
                cv2.imshow('Original', record_video.cv_image)
                cv2.imshow('Thermal', record_video.mask)
                cv2.imshow('Thermal', record_video.resulted_image)
                cv2.imshow('Corrosion', record_video.cv_image_corrosion)

            except Exception as e:
                try:
                    rospy.logerr(str(traceback.format_exc()))
                except:
                    sys.exit()
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                # Release everything if job is finished
                out.release()
                cv2.destroyAllWindows()
                break
            rate.sleep()
        except KeyboardInterrupt:
            out.release()
            sys.exit()