#!/usr/bin/env python3

import rospy
import time
from gazebo_msgs.msg import LinkStates, ModelStates
from geometry_msgs.msg import Pose

frequency=0.2 

class GazeboModelPose:
  model_pose = Pose()
  current_pose=Pose()

  def __init__(self, model_name):
    self.model_name = str(model_name)
    self.model_name_rectified = model_name.replace("::", "_")

    if not self.model_name:
      raise ValueError("'model_name' is an empty string")

    self.states_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
    self.pose_pub = rospy.Publisher("edo/object_odometry/" +self.model_name, Pose, queue_size = frequency)

  def callback(self, data):
    try:
      ind = data.name.index(self.model_name)
      self.model_pose = data.pose[ind]
      self.current_pose=data.pose

    except ValueError:
      pass


if __name__ == '__main__':
  
  rospy.init_node('pose_detection', anonymous=True)
  object_pose=GazeboModelPose('coke_can') 
  rate = rospy.Rate(frequency)

  while not rospy.is_shutdown():
    object_pose.pose_pub.publish(object_pose.model_pose)
    
    rate.sleep()

