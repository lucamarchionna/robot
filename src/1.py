#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
from std_msgs.msg import String, Float64, Int32
from edocontroller import EdoFactory, REAL_ROBOT
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from moveit_msgs.msg import RobotState, Constraints, OrientationConstraint
from obstacle_avoidance import obstacle_AttachAdd, clean_scene
from quater import camera2robot

# initialize the controller wanted, here the one used along with the simulation
controller = EdoFactory().create_robot_model(REAL_ROBOT)
scene=controller.scene
ee_link=controller.eeLink

def webcam2baselink(pose):
  raggio=0.042
  pose.position.x= (pose.position.x-0.657-raggio)
  pose.position.z=pose.position.z+0.438
  qy=pyquaternion.Quaternion(axis=[0,1,], angle=math.pi/2)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  qz=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/2)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)



def left_orientation(data):
  targetPose=PoseStamped()
  offset=0.05

  rospy.loginfo("This is the x-value: %f" %data.position.x)
  rospy.loginfo("This is the y-value: %f" %data.position.y)
  rospy.loginfo("This is the z-value: %f" %data.position.z)

  targetPose.header.stamp=rospy.Time.now()
  targetPose.header.frame_id="edo_base_link"
  targetPose.pose.position=data.position
  targetPose.pose.position.x=data.position.x #-numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=data.position.y #+numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=data.position.z

  approach_x=targetPose.pose.position.x
  approach_y=targetPose.pose.position.y
  """
  q1=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
  q3=q1*q2
  targetPose.pose.orientation.x= q3.x #data.orientation.x
  targetPose.pose.orientation.y= q3.y  #data.orientation.y
  targetPose.pose.orientation.z= q3.z  #data.orientation.z
  targetPose.pose.orientation.w= q3.w  #data.orientation.w
  """
  #set up edo settings
  #obstacle_AttachAdd()
  controller.set_velocity(0.3)
  controller.set_target(targetPose)
  result=controller.trajectory_status()
  controller._go_to_moveit_pose_goal(targetPose)

  return result, targetPose


def left_approach(targetPose):
  offset=0.08
  rospy.sleep(3)
  clean_scene()
  rospy.sleep(2)

  rospy.loginfo("Begin the approaching to the i-th block")
  q1=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
  q3=q1*q2
  targetPose.header.frame_id="edo_base_link"
  targetPose.header.stamp=rospy.Time.now()

  targetPose.pose.orientation.x= q3.x #data.orientation.x
  targetPose.pose.orientation.y= q3.y  #data.orientation.y
  targetPose.pose.orientation.z= q3.z  #data.orientation.z
  targetPose.pose.orientation.w= q3.w  #data.orientation.w

  targetPose.pose.position.x=targetPose.pose.position.x +numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=targetPose.pose.position.y -numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=targetPose.pose.position.z

  controller.set_velocity(0.2)
  #controller.approach_block_straightaway(targetPose)
  controller.set_target(targetPose)
  rospy.sleep(3)
  controller._go_to_moveit_pose_goal(targetPose)
  rospy.sleep(3)

  targetPose.header.stamp=rospy.Time.now()
  targetPose.pose.position.x=targetPose.pose.position.x -numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=targetPose.pose.position.y +numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=targetPose.pose.position.z
  controller.set_velocity(0.2)
  #controller.approach_block_straightaway(targetPose)
  controller.set_target(targetPose)
  rospy.sleep(3)
  controller._go_to_moveit_pose_goal(targetPose)


def right_orientation(data):
  targetPose=PoseStamped()
  offset=0.05

  targetPose.header.frame_id="edo_base_link"
  targetPose.pose.position=data.position
  targetPose.pose.position.x=data.position.x -numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=data.position.y +numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=data.position.z

  q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
  q3=q1*q2
  targetPose.pose.orientation.x= q3.x #data.orientation.x
  targetPose.pose.orientation.y= q3.y  #data.orientation.y
  targetPose.pose.orientation.z= q3.z  #data.orientation.z
  targetPose.pose.orientation.w= q3.w  #data.orientation.w

  #set up edo settings
  obstacle_AttachAdd()
  controller.set_velocity(0.99)
  controller.set_target(targetPose)
  result=controller.trajectory_status()
  controller._go_to_moveit_pose_goal(targetPose)

  return result, targetPose

def right_approach(targetPose):
  offset=0.05
  rospy.sleep(3)
  clean_scene()
  rospy.sleep(2)

  rospy.loginfo("Begin the approaching to the i-th block")
  q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
  q3=q1*q2
  targetPose.header.frame_id="edo_base_link"
  targetPose.pose.orientation.x= q3.x #data.orientation.x
  targetPose.pose.orientation.y= q3.y  #data.orientation.y
  targetPose.pose.orientation.z= q3.z  #data.orientation.z
  targetPose.pose.orientation.w= q3.w  #data.orientation.w

  targetPose.pose.position.x=targetPose.pose.position.x +numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=targetPose.pose.position.y -numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=targetPose.pose.position.z

  controller.set_velocity(0.2)
  #controller.approach_block_straightaway(targetPose)
  controller.set_target(targetPose)
  controller._go_to_moveit_pose_goal(targetPose)
  rospy.sleep(3)

  targetPose.pose.position.x=targetPose.pose.position.x -numpy.sign(targetPose.pose.position.x)*offset
  targetPose.pose.position.y=targetPose.pose.position.y +numpy.sign(targetPose.pose.position.y)*offset
  targetPose.pose.position.z=targetPose.pose.position.z
  controller.set_velocity(0.2)
  #controller.approach_block_straightaway(targetPose)
  controller.set_target(targetPose)
  controller._go_to_moveit_pose_goal(targetPose)
  

if __name__ == "__main__":
  rospy.init_node("simple joint control", anonymous=True)
  rospy.sleep(5)

  def posCallback(data):
    targetPose=PoseStamped()
    offset=0.05
    
    #data2=camera2robot(data)
    result, targetPose = left_orientation(data)
    
    """
    if result==True:
      left_approach(targetPose)

    
    else:
      rospy.loginfo("Cannot reach the desired point")
   """
  
  def forceCallback(force):
    if force.data>0.2:
      controller.stop()

  


  sub=rospy.Subscriber('object_odometry/blockposition',Pose,posCallback)
  subForce=rospy.Subscriber('object_odometry/blockposition',Pose,forceCallback)
  rospy.spin()


""" Different functions
    #controller.shift(0.3)
    #controller._go_to_moveit_pose_goal(targetPose) 
"""