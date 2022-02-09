#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped


def camera2robot(data):
    pose_robot=Pose()
    qy=pyquaternion.Quaternion(axis=[0,1,0], angle=math.pi/2)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
    qz=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/2)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)

    q_robot=qy*qz
    transformation_camera2robot = q_robot.transformation_matrix

    raggio = 0.042
    transl = numpy.array([-0.657-raggio,0,0.438])

    transformation_camera2robot[:3,3] = transl.reshape(3)

    vector=numpy.array([ [data.position.x], [data.position.y], [data.position.z] ,[1] ])
    vector_robot=numpy.dot(transformation_camera2robot, vector)

    pose_robot.position.x=vector_robot[0,0]
    pose_robot.position.y=vector_robot[1,0]
    pose_robot.position.z=vector_robot[2,0]

    pose_robot.orientation.x=0
    pose_robot.orientation.y=0
    pose_robot.orientation.z=0
    pose_robot.orientation.w=1

    return pose_robot
    
if __name__ == "__main__":
    robot=Pose()
    robot.position.x=0.125
    robot.position.y=-0.1
    robot.position.z=0.53

    robot=camera2robot(robot)
    print("" ,robot)

