#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from mando_yolo import Mando

rospy.init_node('mando_main')

mando = Mando()

while not rospy.is_shutdown():
    mando.control()
