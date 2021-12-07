#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import cv2, random, math
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from matplotlib import pyplot as plt


class TSign():

    def __init__(self):
        self.sift = cv2.xfeatures2d.SIFT_create()	              # SIFT Detector 초기화

        # FLANN parameters
        self.FLANN_INDEX_KDTREE = 0
        self.index_params = dict(algorithm = self.FLANN_INDEX_KDTREE, trees = 5)
        self.search_params = dict(checks=50)   # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(self.index_params,self.search_params)

        self.bridge = CvBridge()
        self.WIDTH = 640
        self.HEIGHT = 480

        self.image = np.empty(shape=[0])
        self.image_sub = rospy.Subscriber("csi_image", Image, self.img_callback)
        self.status = 0


    def img_callback(self, data):
        self.image = cv2.resize(self.bridge.imgmsg_to_cv2(data, "bgr8"), (self.WIDTH, self.HEIGHT))


    def FeatureDetect(self, des1, num):

        match_cnt = 0
        sign_path = '/home/srko/바탕화면/jjjj/src/Signs/%d.JPG' % num
        sign = cv2.imread(sign_path, 0)
        sign_blur = cv2.blur(sign, (5,5))
        # find the keypoints and descriptors with SIFT
        kp2, des2 = self.sift.detectAndCompute(sign_blur, None)

        matches = self.flann.knnMatch(des1,des2,k=2)

        # Need to draw only good matches, so create a mask
        matchesMask = [[0,0] for i in range(len(matches))]

        # ratio test as per Lowe's paper
        for i,(m,n) in enumerate(matches):
            if m.distance < 0.6 * n.distance:
                matchesMask[i]=[1,0]

        for match in matchesMask:
            if match[0] == 1:
                match_cnt += 1

        if match_cnt > 4:
            self.status = num


    def Detect(self):

        self.status = 0

        while not self.image.size == (self.WIDTH * self.HEIGHT * 3):
            return

        cv2.imshow("Image", self.image)

        gray = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        blur = cv2.blur(blur, (5,5))

        # find the keypoints and descriptors with SIFT
        kp1, des1 = self.sift.detectAndCompute(blur, None)

        for num in range(1, 8):
        self.FeatureDetect(des1, num)
        

