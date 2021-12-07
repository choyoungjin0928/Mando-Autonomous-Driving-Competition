#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import pickle
import time
from stanley import StanleyControl

from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import Float64, Int32
from sensor_msgs.msg import Imu

imu_flag = False
num = 0
class StanleyController(object):
    def __init__(self):
        self.rear_x = 0.0
        self.rear_y = 0.0
        self.yaw = 0.0
        self.pitch = 0.0
        self.roll = 0.0
        self.v = 0
        self.speed = 0
        self.angle = 135
        self.index = 0
        self.yaw_init =0.0
        self.yaw_term =0.0
        self.line_cte = 0.0
        self.rear_x_previous = 0.0
        self.rear_y_previous = 0.0
        self.previous_time = time.time()
        self.idx_chk = 0
        
        with open("/home/amap/aMAP_catkin_ws/src/xycar_slam/maps/mando_11_27_2.pkl", "rb") as f:
            self.path = pickle.load(f)
        
        self.tracked_pose_sub = rospy.Subscriber("tracked_pose", PoseStamped, self.PoseCallBack)
        self.ego_imu_sub = rospy.Subscriber("handsfree/imu", Imu, self.ImuCallBack)

    def ImuCallBack(self, msg):
        global imu_flag
          
        self.imu_data = msg.linear_acceleration.z
        
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        ])
        #self.yaw = -(self.yaw / 1000)
        if(not imu_flag):
          self.yaw_init = self.yaw
          
          imu_flag = True
#        print("yaw_init: ", self.yaw_init)
#        self.yaw -= self.yaw_init
        self.yaw = -(self.yaw - self.yaw_init)
#        self.yaw = -(self.yaw + 1.6)
        #print("yaw: ", self.yaw)
    
    def PoseCallBack(self, msg):
        self.rear_x = msg.pose.position.x
        self.rear_y = msg.pose.position.y
        orientation_q = msg.pose.orientation
        """
        self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w
        ])
        """
        """
        dx = self.rear_x - self.rear_x_previous
        dy = self.rear_y - self.rear_y_previous
        dist = np.hypot(dx, dy)

        present_time = time.time()
        tact_time = present_time - self.previous_time

        print('v : ', float(dist) / tact_time) # 0.35
        self.previous_time = present_time
        self.rear_x_previous = self.rear_x
        self.rear_y_previous = self.rear_y
        """
        self.v = 0.2
        

    def Control(self):
      	#self.yaw +=1.0
        #print("yaw: ", self.yaw)
        #print("yaw: ", self.yaw)
        #print("path yaw : ", self.path['yaw'])
        delta, idx, self.yaw_term = StanleyControl(self.rear_x, self.rear_y, self.yaw, self.v, self.index, 
                               self.path['x'], self.path['y'], self.path['yaw'], 0.09, 0.8)

        self.index = idx
        print(self.index, " : ", delta)
        if delta < -18.0775947922:
            delta = -18.0775947922
        elif delta > 18.0775947922:
            delta = 18.0775947922

        angle = int(delta * 40 / 18.0775947922) +135
      
        if angle < 95:
            angle = 95
        elif angle > 175:
            angle = 175


        self.angle = angle
        self.speed = 180
        
        if self.index / 1000 != self.idx_chk:
            self.speed = 0
            self.idx_chk = self.index / 1000
            print(self.index)
        
