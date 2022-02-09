#!/usr/bin/env python3

import math
import re
from turtle import pos
from urllib import request
import numpy
import pyquaternion 
import time
import rospy
import tf
import numpy as np
from std_msgs.msg import Bool
from tracker_visp.msg import *
from tracker_visp.srv import *

def FirstLayerPoseCallback():

    rospy.wait_for_service('FirstLayerPose')
    pose = ReferenceBlock()
    pose.location.layer = 18
    pose.location.orientation = 'dx'
    pose.location.position = 'dx'

    pose.pose.pose.position.x = 0.4 #0.075*math.cos(math.pi/4)
    pose.pose.pose.position.y = -0.20
    pose.pose.pose.position.z = 0.60
    pose.pose.pose.orientation.x = 0.0
    pose.pose.pose.orientation.y = -0.3826834
    pose.pose.pose.orientation.z = 0.0
    pose.pose.pose.orientation.w = 0.9238795

    fan = Bool()
    fan.data = False

    for i in range(2):

        req = rospy.ServiceProxy('FirstLayerPose', FirstLayerPose)
        abc = req(fan, pose)
        fan.data = True
    return {'found':fan, 'cTlayer1':pose}




if __name__ == "__main__":
    rospy.init_node("response_service")
    req = Bool()
    FirstLayerPoseCallback()
    rospy.spin()
