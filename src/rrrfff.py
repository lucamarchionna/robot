#!/usr/bin/env python3

import math
import string
from matplotlib.transforms import Transform
import numpy
import pyquaternion 
import time
import rospy
import tf
import tf2_ros
import numpy as np
from edo_core_msgs.msg import JointControlArray, JointControl, JointStateArray
from edocontroller import EdoFactory, REAL_ROBOT
from std_msgs.msg import String, Float32, Int32, Float64, Bool
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Quaternion, TransformStamped
from sensor_msgs.msg import JointState
from obstacle_avoidance import obstacle_AttachAdd, clean_scene
from velocityController import VelocityController
from positionbased_vs.srv import InitialGuess
from tracker_visp.msg import *
from tracker_visp.srv import *


def BlockEstimationCallback():
    global bTo_top, Flag_threshold

    Flag_threshold = False
    location_top = ['sx', 'dx', 18]
    position_top = location_top[1]
    orientation_top = location_top[2]
    layer_top = location_top[3]

    position = 'sx'
    orientation = 'dx'
    layer= 12

    params = [position_top, position, orientation_top, orientation, layer_top, layer]
    
    bTo_est = compute_bTo(params)

    #posblockCallback(bTo_est) ## Motion

    cTo_est = tocTo(bTo_est)

    cTo_est = toReference(cTo_est)

    cTo_est.location.position = position
    cTo_est.location.orientation = orientation
    cTo_est.location.layer = int(layer)

    return BlockEstimationResponse(
        cTo_est
    )

def compute_bTo(geometric_params):

    global bTo_top
 
    bTo = ReferenceBlock()
    bTo.location.position = geometric_params[1]
    bTo.location.orientation = geometric_params[3]
    bTo.location.layer = geometric_params[5]

    bTo_topmost = np.array([ 
        [1, 0, 0, 0],
        [0, 1, 0, 0.1],
        [0, 0, 1, 0.8],
        [0, 0, 0, 1],
    ])
    bTo.pose = toPoseMsg(bTo_topmost)
    
    diff_layer = float(geometric_params[4]) - geometric_params[5]

    q_z = pyquaternion.Quaternion(axis = [0, 0, 1], angle = 0)

    n_blocks = 0.0
    sign_block = 1.0
    change_orient = 0.0
    z_blocks = 3.0

    if geometric_params[2] == 'dx' and geometric_params[3] == 'dx':
        pass

    elif geometric_params[2] == 'dx' and geometric_params[3] == 'sx':
        change_orient = 1.0
        
        bTo.pose.pose.position.y = -bTo.pose.pose.position.y

        q_z = pyquaternion.Quaternion(axis = [0, 0, 1], angle = -math.pi/2)
        q_final = quat_molt(q_z, bTo.pose)
        bTo.pose.pose.orientation = q_final.pose.orientation

    elif geometric_params[2] == 'sx' and geometric_params[3] == 'sx':
        pass

    elif geometric_params[2] == 'sx' and geometric_params[3] == 'dx':
        change_orient = 1.0

        bTo.pose.pose.position.y = -bTo.pose.pose.position.y

        q_z = pyquaternion.Quaternion(axis = [0, 0, 1], angle = math.pi/2)
        q_final = quat_molt(q_z, bTo.pose)
        bTo.pose.pose.orientation = q_final.pose.orientation

    else:
        rospy.logwarn("Provide a right convention's name")


    if geometric_params[0] == 'sx' and geometric_params[1] == 'sx':
        z_blocks = 3.0
        pass

    elif geometric_params[0] == 'dx' and geometric_params[1] == 'dx':
        pass    

    elif geometric_params[0] == 'cx' and geometric_params[1] == 'cx':
        pass    

    elif geometric_params[0] == 'sx' and geometric_params[1] == 'dx':
        n_blocks = 2.0
        sign_block = 1.0

    elif geometric_params[0] == 'dx' and geometric_params[1] == 'sx':
        n_blocks = 2.0
        sign_block = -1.0

    elif geometric_params[0] == 'cx' and geometric_params[1] == 'dx':
        n_blocks = 1.0
        sign_block = 1.0

    elif geometric_params[0] == 'dx' and geometric_params[1] == 'cx':
        n_blocks = 1.0
        sign_block = -1.0

    elif geometric_params[0] == 'cx' and geometric_params[1] == 'sx':
        n_blocks = 1.0
        sign_block = -1.0

    elif geometric_params[0] == 'sx' and geometric_params[1] == 'cx':
        n_blocks = 1.0
        sign_block = 1.0

    else:
        rospy.logwarn("Provide a right convention's name")
    
    p1Tp2 = np.array([sign_block*n_blocks*0.025, diff_layer*0.015, change_orient*0.5])
    p1Tp2_mat = toHomogeneousMatrix(p1Tp2)

    bTo = toHomogeneousMatrix(bTo.pose.pose)

    bTo = bTo.dot(p1Tp2_mat)

    bTo_block = toTransformStampedMsg(bTo)
    bTo_block.child_frame_id = "block "+ geometric_params[1] + " " + geometric_params[3] + " " + str(geometric_params[5]) 

    br.sendTransform(bTo_block)

    return bTo
    


if __name__ == "__main__":


    bTo = PoseStamped()
    bTo.header = "block 18 sx dx"
    bTo.pose.position.z = 0.4
    
    p1Tp2 = np.array([0, 0, 0])

    print(p1Tp2.shape[0])



    name = bTo.header

    parts = bTo.header.split()

    numb = parts[1]

    print(type(numb))
    print(numb)

    numb = float(numb)
    print(type(numb))
    print(numb)

    # tfBuffer = tf2_ros.Buffer()

    # listener = tf.TransformListener()

    # rospy.sleep(5)
    # callback(bTo)


    # rospy.spin()
