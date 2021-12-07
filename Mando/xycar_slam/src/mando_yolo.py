#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import rospy
import time
import numpy as np
from std_msgs.msg import UInt16
from std_msgs.msg import Int16
from stanley_follower import StanleyController
from yolo import YOLO


class Mando(object):

    def __init__(self):
        self.rate = rospy.Rate(10)

        self.dc_pub = rospy.Publisher("pub_dc", Int16, queue_size=1)
        self.servo_pub = rospy.Publisher("pub_servo", UInt16, queue_size=1)
        self.speed = 0
        self.angle = 135

        self.stanley_follower = StanleyController()
        self.yolo = YOLO()
        
        self.stopline1 = True
        self.stopline2 = True
        self.stopline3 = True
        self.stopline4 = True
        self.stopline5 = True
        self.last = True
        
        self.time_flag = False

    def control(self):

        self.stanley_follower.Control()

        # Tracking(Stanley Method)가 속도와 조향각의 기본 베이스로 들어간다.
        self.speed = self.stanley_follower.speed
        self.angle = self.stanley_follower.angle
        
        # 정지선 정지
        if (1800 < self.stanley_follower.index < 1900) and self.stopline1:      # 1420~1500
            self.stop()
            time.sleep(2)
            self.stopline1 = False
        if (11850 < self.stanley_follower.index < 11900) and self.stopline2:
            self.stop()
            self.stopline2 = False        
        if (15760 < self.stanley_follower.index < 15800) and self.stopline3:
            self.stop()
            self.stopline3 = False
        if (22400 < self.stanley_follower.index < 22450) and self.stopline4:
            self.stop()
            self.stopline4 = False
        if (30980 < self.stanley_follower.index < 31500) and self.stopline5:
            self.stop()
            self.stopline5 = False
        """
        if (self.stanley_follower.index > 34200) and self.reverse:
            self.drive(180, 105)
            time.sleep(1)
            self.stop()
            self.drive(-250, 165)
            time.sleep(8)
            self.reverse = False
        """
        
        if (17000 < self.stanley_follower.index < 18000) or (28000 < self.stanley_follower.index < 30950):
            self.angle -= 5
            
        if self.stanley_follower.index < 1800:                                  # 1450
            self.angle = 135
            
                    
        if self.yolo.check:
            self.yolo.check = False
            if self.yolo.Red_B:
                print("stop sign")
                self.yolo.Red_B = False
                self.stop()
                self.time_flag = True
        
        if (self.stanley_follower.index > 31000) and self.last:
            self.last = False
            self.drive(180, 165)
            time.sleep(3)
            
        if (9000 < self.stanley_follower.index < 11850):    # 6400~9000
            self.speed = 100
            
        if not self.last:
            self.speed = 0
                        
        self.drive(self.speed, self.angle)
        self.rate.sleep()

    def stop(self):
        self.drive(0, 135)
        time.sleep(3)
        
    def drive(self, Angle, Speed):
        self.speed = Angle
        self.angle = Speed
        self.dc_pub.publish(self.speed)
        self.servo_pub.publish(self.angle)
        
        