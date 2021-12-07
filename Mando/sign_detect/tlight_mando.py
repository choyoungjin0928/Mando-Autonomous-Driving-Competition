#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import time
import numpy as np
from std_msgs.msg import UInt16
from stanley_follower import StanleyController


class Mando(object):

    def __init__(self) :
        self.rate = rospy.Rate(10)

        self.dc_pub = rospy.Publisher("pub_dc", UInt16, queue_size=1)
        self.servo_pub = rospy.Publisher("pub_servo", UInt16, queue_size=1)
        self.speed = 0
        self.angle = 135

        self.stanley_follower = StanleyController()


    def control(self) :

        self.stanley_follower.Control()

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.speed = self.stanley_follower.speed
        self.angle = self.stanley_follower.angle

        self.dc_pub.publish(self.speed)
        self.servo_pub.publish(self.angle)

        self.rate.sleep()
