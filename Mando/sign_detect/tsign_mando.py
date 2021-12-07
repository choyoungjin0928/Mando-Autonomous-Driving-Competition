#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import time
import numpy as np
from std_msgs.msg import UInt16
from TSign import TSign


class Mando(object):

    def __init__(self):
        self.rate = rospy.Rate(10)

        self.dc_pub = rospy.Publisher("pub_dc", UInt16, queue_size=1)
        self.servo_pub = rospy.Publisher("pub_servo", UInt16, queue_size=1)
        self.speed = 0
        self.angle = 135

        self.t_sign = TSign()


    def control(self):

        self.t_sign.Detect()

        if self.t_sign.status == 1:
            print("직진")
        elif self.t_sign.status == 2:
            print("우회전")
        elif self.t_sign.status == 3:
            print("좌회전")
        elif self.t_sign.status == 4:
            print("주차")
        elif self.t_sign.status == 5:
            print("정지")
        elif self.t_sign.status == 6:
            print("주차금지")
        elif self.t_sign.status == 7:
            print("횡단보도")

        self.rate.sleep()
