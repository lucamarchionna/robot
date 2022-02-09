#!/usr/bin/env python3

import socket
import time
import os
import rospy

PACKAGE='edo_core_pkg'
import roslib
#roslib.load_manifest(PACKAGE)

#from dynamic_reconfigure.server import Server as DynamicReconfigureServer

import std_msgs.msg

from edo_core_msgs.msg import JointControl
from edo_core_msgs.msg import MovementCommand
from edo_core_msgs.msg import JointControlArray
from edo_core_msgs.msg import MovementFeedback
from edo_core_msgs.msg import CartesianPose
from edo_core_msgs.msg import Point
from edo_core_msgs.msg import Frame

from edo_core_msgs.srv import ControlSwitch

from parse import parse

algo_jnt_ctrl = rospy.Publisher('algo_jnt_ctrl', JointControlArray, queue_size=1)
machine_move = rospy.Publisher('bridge_move', MovementCommand, queue_size=1)

waitme = 0
def callback_algo_movement_ack(ack):
  global waitme

  print("callback algo movement ack")
  if ack.type == 2:
    waitme = 1

rospy.init_node('test', anonymous=True)
rate = rospy.Rate(100) # 100hz

algo_movement_ack = rospy.Subscriber('algo_movement_ack', MovementFeedback, callback_algo_movement_ack)

if __name__ == '__main__':
  print("Python Test")


  joints_via = [
    float(0),
    float(0),
    float(0),
    float(0),
    float(0),
    float(0),
    float(0),
    ]

  joints = [
    float(0),
    float(0),
    float(0),
    float(0.00),
    float(0),
    float(0),
    float(1.84),
    ]
        
  ros_cartesian_pose = CartesianPose(0.0,0.0,0.0,0.0,0.0,0.0,'')
  ros_point =  Point(74, ros_cartesian_pose, 127, joints)
  ros_via   =  Point(0,ros_cartesian_pose,0,joints_via)
  ros_frame =  Frame(0.0,0.0,0.0,0.0,0.0,0.0)

  mmmsg = MovementCommand(77, 74, 100, 0, 0, 0.0, ros_point, ros_via, ros_frame, ros_frame)
  machine_move.publish(mmmsg)

  print("Point1")
  rospy.sleep(10)

  joints = [
    float(0),
    float(0),
    float(0),
    float(0),
    float(0),
    float(0),
    float(1.84),
    ]
        
  ros_cartesian_pose = CartesianPose(0.0,0.0,0.0,0.0,0.0,0.0,'')
  ros_point =  Point(74, ros_cartesian_pose, 127, joints)
  ros_via   =  Point(0,ros_cartesian_pose,0,joints_via)
  ros_frame =  Frame(0.0,0.0,0.0,0.0,0.0,0.0)

  mmmsg = MovementCommand(77, 74, 100, 0, 0, 0.0, ros_point, ros_via, ros_frame, ros_frame)
  machine_move.publish(mmmsg) 
  print("Point2")	

  #publish move
  #machine_move.publish(mmmsg)
