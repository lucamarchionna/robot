#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
from edocontroller import EdoFactory, SIMULATION
from std_msgs.msg import String, Float32, Int32
from geometry_msgs.msg import Pose, PoseStamped
from obstacle_avoidance import obstacle_AttachAdd, clean_scene



controller = EdoFactory().create_robot_model(SIMULATION)


def insertdataPos(data, q3):
  robotposition=PoseStamped()
  robotposition.header.stamp=rospy.Time.now()
  robotposition.header.frame_id="edo_base_link"
  robotposition.pose.position.x=data.pose.position.x
  robotposition.pose.position.y=data.pose.position.y
  robotposition.pose.position.z=data.pose.position.z
  
  robotposition.pose.orientation.x= q3.x #data.orientation.x
  robotposition.pose.orientation.y= q3.y  #data.orientation.y
  robotposition.pose.orientation.z= q3.z  #data.orientation.z
  robotposition.pose.orientation.w= q3.w  #data.orientation.w
  
  #obstacle_AttachAdd()
  return robotposition

def pushblock(data, offset):
  approachblock=PoseStamped()
  approachblock=data
  approachblock.pose.position.x=data.pose.position.x-offset
  approachblock.pose.position.y=data.pose.position.y+offset
  approachblock.pose.position.z=data.pose.position.z

  result=asynAction(approachblock, 0.1)
  return result

def retract(data, offset):
  approachblock=PoseStamped()
  approachblock=data
  approachblock.pose.position.x=data.pose.position.x+offset
  approachblock.pose.position.y=data.pose.position.y-offset
  approachblock.pose.position.z=data.pose.position.z
  result=asynAction(approachblock, 0.1)
  return result

def action(Pose2reach, velocity):
  controller.set_velocity(velocity)
  controller.set_target(Pose2reach)
  resultTraj=controller.trajectory_status()
  controller._go_to_moveit_pose_goal(Pose2reach)
  rospy.sleep(2)
  return resultTraj

def asynAction(Pose2reach, velocity):
  controller.set_velocity(velocity)
  controller.set_target(Pose2reach)
  resultTraj=controller.trajectory_status()
  #controller.approach_block_straightaway(targetPose)
  controller.asynchronousMovement(Pose2reach)
  #rospy.sleep(2)
  return resultTraj


def actionblock(Pose2reach, velocity):

  controller.set_target(Pose2reach)
  #controller.approach_block_straightaway(Pose2reach)
  controller.set_velocity(velocity)
  resultTraj=controller.trajectory_status()
  controller._go_to_moveit_pose_goal(Pose2reach)

  return resultTraj


Flag=False

if __name__ == "__main__":
  simplcontrol=rospy.init_node("simplcontrol")
  rospy.sleep(5)
  
  q1=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
  q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
  qleft=q1*q2
  q_calib=pyquaternion.Quaternion(axis=[0,0,1], angle=0) 
  rospy.loginfo(q_calib)
  #movement2 = rospy.Publisher('object_odometry/blockposition', PoseStamped, queue_size=10, latch=True)
  
  def posCallback(data):
      
    offset=0.05
    targetPose=insertdataPos(data, q_calib)
    result=action(targetPose, 0.6)
    rospy.sleep(2)
    k=0
    """
    while result==True:
      targetPose.pose.position.x=targetPose.pose.position.x+0.005
      targetPose.pose.position.y=targetPose.pose.position.y-0.005
      rospy.loginfo("Iteration number: %d" %k)
      rospy.loginfo("x value: %f " %targetPose.pose.position.x)
      rospy.loginfo("x value: %f   y-value: %f z-value: %f" ,targetPose.pose.position.x, targetPose.pose.position.y, targetPose.pose.position.z)
      result=action(targetPose, 0.1)
      k=k+1


  
    if result==True:
      clean_scene()
      result=pushblock(targetPose, offset)


    else:
      rospy.loginfo("Cannot reach the desired point")
    """
      

  def forceCallback(force):
    global Flag
    if force.data>0.2:
      if not Flag:
        Flag=True
        controller.stop()
        pose4retract=controller.get_current_pose()
        retract(pose4retract, 0.03)
        #rospy.sleep(2)
    
    else:
      Flag=False
      pass
          
          

  sub=rospy.Subscriber('object_odometry/blockposition',PoseStamped,posCallback)
  subForce=rospy.Subscriber('/appliedForce',Float32,forceCallback)
  rospy.spin()


"""
pushmov=pushblock(targetPose, offset)
controller.set_velocity(0.1)
controller.approach_block_straightaway(pushmov)
controller.set_target(pushmov)
controller._go_to_moveit_pose_goal(pushmov)



approachblock=PoseStamped()
approachblock.pose.position.x=targetPose.pose.position.x+offset
approachblock.pose.position.y=targetPose.pose.position.y-offset
approachblock.pose.position.z=targetPose.pose.position.z
movement2.publish(approachblock)
"""