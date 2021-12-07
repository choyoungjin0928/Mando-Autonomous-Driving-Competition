#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from tlight_mando import Mando

rospy.init_node('mando_main')

mando = Mando()

while not rospy.is_shutdown():
    Mando.control()
