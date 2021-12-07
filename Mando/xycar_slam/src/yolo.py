#! /usr/bin/env python
#-*- coding: utf-8 -*-
import rospy, math
import cv2, time, rospy
import numpy as np

from darknet_ros_msgs.msg import BoundingBoxes


class YOLO():

    def __init__(self):

        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.callback_box)
        
        self.boxdata = None
        self.check = False
        self.Red_A = False
        self.Red_B = False
        self.Red_C = False
        self.Red_D = False
        self.Blue_A = False
        self.Blue_B = False


    def callback_box(self, msg):

        self.boxdata = msg

        if self.boxdata is not None:
            self.check = True
            for i in self.boxdata.bounding_boxes:
                if i.Class == "Red_A":
                    self.Red_A = True
                    break
                if i.Class == "Red_B":
                    self.Red_B = True
                    break
                if i.Class == "Red_C":
                    self.Red_C = True
                    break
                if i.Class == "Red_D":
                    self.Red_D = True
                    break
                if i.Class == "Blue_A":
                    self.Blue_A = True
                    break
                if i.Class == "Blue_B":
                    self.Blue_B = True
                    break
    

