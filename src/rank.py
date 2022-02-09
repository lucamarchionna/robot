#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
import tf
from edocontroller import EdoFactory, REAL_ROBOT
from edo_core_msgs.msg import JointInit, CollisionThreshold, MovementCommand, CartesianPose, Point
import numpy as np
from edo_core_msgs.msg import JointControlArray, JointControl, JointStateArray
from std_msgs.msg import String, Float32, Int32, Float64, Bool
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped

def waypoints(joint_values, points):
    js_waypoints=np.zeros((points, 6), dtype=float)

    for k in range(6):
        i=0

        for i in range(points):
            js_waypoints[i][k]=joint_values[k]*(i+1)/points
   
    
        


if __name__ == "__main__":
    init_node=rospy.init_node("rank", anonymous=True)
    
    controller = EdoFactory().create_robot_model(REAL_ROBOT)

    final_js=np.zeros(6)

    rate = rospy.Rate(10)
    js=np.ones(6)
    waypoints(js, 5)
    rospy.spin()

    while not rospy.is_shutdown():
        J=controller.jacobian()
        rankJ=np.linalg.matrix_rank(J)

        if (np.linalg.det(J)):
            rospy.loginfo("Rank is : %d", rankJ)
            rospy.loginfo(J)


        else:
            rospy.loginfo("Beccato \n")
            rospy.loginfo("Rank is : %d", rankJ)

        inv_J=np.linalg.inv(J)


        rate.sleep()

    