#!/usr/bin/env python

import roslib
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import torch
import time
import numpy as np
import os, time, signal, threading
from subprocess import call
from subprocess import Popen
import subprocess


model = torch.hub.load('src/yolov5', 'custom', path='src/teleop_twist_keyboard/best.pt', source='local')
pic_path1 = '/home/huy/catkin_ws/src/teleop_twist_keyboard/camera_image1.jpeg'
pic_path2 = '/home/huy/catkin_ws/src/teleop_twist_keyboard/camera_image2.jpeg'

pics = [pic_path1, pic_path2]

bridge = CvBridge()
proc_movebase = None
proc_explore = None
flag = 0
total_count = 0


def image_callback1(msg):
    convert_image(msg, 1)

def image_callback2(msg):
    convert_image(msg, 2)

def convert_image(msg, img_num):
    try:
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except e:
        print(e)
    else:
        cv2.imwrite(pics[img_num - 1], cv2_img)
        runYolo(pics[img_num - 1], img_num)


def runYolo(pic, drone_num):

    results = model(pic)

    if results.pandas().xyxy[0].loc[:,'confidence'].empty:
        print("No target seen :(\n")
    else:
        if results.pandas().xyxy[0].loc[:,'confidence'].values >= 0.40:
            # global flag
            # if (flag == 1):
            #     end_pathfinding()
            #     flag = 0
                
                
            print("The target is in view!! and the pandas DF looks like this\n", results.pandas().xyxy[0], "\n")
            global total_count
            total_count += 1
            if (total_count % 30 == 0):
                results.show()
        else:
            print("No target seen :(\n")

def start_pathfinding():
    global proc_movebase 
    global proc_explore 
    proc_movebase = subprocess.Popen(["roslaunch", "quadrotor_navigation", "quadrotor_move_base.launch"])
    proc_explore = subprocess.Popen(["roslaunch", "explore_lite", "explore.launch"])

def end_pathfinding():
    os.system("rosnode kill '/explore'")
    os.system("rosnode kill '/move_base'")

def open_world():
    proc_movebase = subprocess.Popen(["roslaunch", " multiple_quadrotors_sim", "main.launch"])

def main():
    # # Start explore lite
    # start_pathfinding()
    # global flag
    # flag = 1

    rospy.init_node('image_listener')

    image_topic1 = "/uav1/front_cam/camera/image"
    image_topic2 = "/uav2/front_cam/camera/image"

    rospy.Subscriber(image_topic1, Image, image_callback1)
    rospy.Subscriber(image_topic2, Image, image_callback2)

    rospy.spin()

if __name__ == '__main__':
    main()