#!/usr/bin/env python

import math
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
# from obstacle_avoidance import obstacle_AttachAdd, clean_scene
from velocityController import VelocityController
from positionbased_vs.srv import InitialGuess
from tracker_visp.msg import *
from tracker_visp.srv import *


controller = EdoFactory().create_robot_model(REAL_ROBOT)
velController = VelocityController("SIM")
Flag=False
Flag_threshold=False
ps=False
frequency=1
global j,k, already_positioned, bTo_top, lastPoseReceived, PoseReceived
PoseReceived = False
j=0
k=0

already_positioned = False
finished = Bool()

def waypoints(joint_values, points):
    js_waypoints=np.zeros((points, 6), dtype=float)

    for k in range(6):
        i=0

        for i in range(points):
            js_waypoints[i][k]=joint_values[k]*(i+1)/points
    
    return js_waypoints

def quat2mat(quat):
    q0=quat[0]
    q1=quat[1]
    q2=quat[2]
    q3=quat[3]
    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    return rot_matrix


def tomatrix(transl, quat):
    
    q0=quat[3]
    q1=quat[0]
    q2=quat[1]
    q3=quat[2]
    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    matrix=np.eye(4, dtype=float)
    matrix[0:3, 0:3]=rot_matrix
    matrix[0:3, 3]=transl

    return matrix
    
def tomatrix_offset(offset, quat):
    
    q0=quat[3]
    q1=quat[0]
    q2=quat[1]
    q3=quat[2]
    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    matrix=np.eye(4, dtype=float)
    matrix[0:3, 0:3] = rot_matrix
    matrix[2, 3] = offset
    matrix[0, 3] = -0.05

    return matrix

def updatefrequency(norm, frequency):
    frequency=round(1+10*norm)
    f_max=30

    
    if frequency>f_max:
        frequency=f_max
    
    else:
        pass

    return frequency

def insertdataMatrix(T_matrix):
    robotposition = PoseStamped()
    robotposition.header.stamp = rospy.Time.now()
    robotposition.header.frame_id = "edo_base_link"
    robotposition.pose.position.x = T_matrix[0, 3]
    robotposition.pose.position.y = T_matrix[1, 3]
    robotposition.pose.position.z = T_matrix[2, 3]

    robotposition.pose.orientation.w = 0.5*math.sqrt(T_matrix[0, 0]+T_matrix[1, 1]+T_matrix[2, 2]+1) #Must be different from 0
    robotposition.pose.orientation.x = (T_matrix[2,1]-T_matrix[1,2])/(4*robotposition.pose.orientation.w)
    robotposition.pose.orientation.y = (T_matrix[0,2]-T_matrix[2,0])/(4*robotposition.pose.orientation.w)
    robotposition.pose.orientation.z = (T_matrix[1,0]-T_matrix[0,1])/(4*robotposition.pose.orientation.w)

    return robotposition

def get_quaternions(T_matrix):
    quat = np.array([1, 0, 0 , 0])

    quat[0] = 0.5*math.sqrt(T_matrix[0, 0]+T_matrix[1, 1]+T_matrix[2, 2]+1) #Must be different from 0
    quat[1] = (T_matrix[2,1]-T_matrix[1,2])/(4*quat[0])
    quat[2] = (T_matrix[0,2]-T_matrix[2,0])/(4*quat[0])
    quat[3] = (T_matrix[1,0]-T_matrix[0,1])/(4*quat[0])

    return quat

def insertdataPos(data, q3):
    robotposition=PoseStamped()
    robotposition.header.stamp=rospy.Time.now()
    robotposition.header.frame_id="edo_base_link"
    robotposition.pose.position.x=data.pose.position.x
    robotposition.pose.position.y=data.pose.position.y
    robotposition.pose.position.z=data.pose.position.z

    if isinstance(q3, pyquaternion.quaternion.Quaternion):
        robotposition.pose.orientation.x= q3.x 
        robotposition.pose.orientation.y= q3.y  
        robotposition.pose.orientation.z= q3.z  
        robotposition.pose.orientation.w= q3.w 

    elif isinstance(q3, numpy.ndarray):
        robotposition.pose.orientation.w= q3[0]
        robotposition.pose.orientation.x= q3[1]
        robotposition.pose.orientation.y= q3[2]
        robotposition.pose.orientation.z= q3[3]
        
    else:
        robotposition.pose.orientation.x= 0
        robotposition.pose.orientation.y= 0  
        robotposition.pose.orientation.z= 0  
        robotposition.pose.orientation.w= 1 


    #obstacle_AttachAdd()
    return robotposition

def go_to_default_pose(pos_x, pos_y, pos_z):
    default = PoseStamped()
    default.pose.position.x = pos_x
    default.pose.position.y = pos_y
    default.pose.position.z = pos_z
    pose2reach = insertdataPos(default, qleft)
    result = action(pose2reach, 0.03)

    return result

def look_right(data):
    q_right=pyquaternion.Quaternion()
    angle=looktower(data)
    q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
    q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2) 
    q_right=q1*q2

    if isinstance(data, numpy.ndarray):
        pose2left=PoseStamped()
        pose2left.pose.position.x=data[0][3]
        pose2left.pose.position.y=data[1][3]
        pose2left.pose.position.z=data[2][3]   
        pose2left.pose.orientation.x=q_right[1]
        pose2left.pose.orientation.y=q_right[2]
        pose2left.pose.orientation.z=q_right[3]
        pose2left.pose.orientation.w=q_right[0]

        pose2left=insertdataPos(pose2left, q_right)
        return pose2left

    else:
        pose=insertdataPos(data, q_right)
        return pose

def looktower(bTt, tTobj):
    bTt.dot(tTobj)
    q_left=pyquaternion.Quaternion()
    angle=angle_obj(bTt, tTobj)
    q1=pyquaternion.Quaternion(axis=[0,0,1], angle=angle)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
    q2=pyquaternion.Quaternion(axis=[0,1,0], angle=math.pi/2) 
    q_left=q1*q2

    data=bTt.dot(tTobj)

    if isinstance(data, numpy.ndarray):
        pose2left=PoseStamped()
        pose2left.pose.position.x = data[0][3]
        pose2left.pose.position.y = data[1][3]
        pose2left.pose.position.z = data[2][3]
        pose2left.pose.orientation.x = q_left[1]
        pose2left.pose.orientation.y = q_left[2]
        pose2left.pose.orientation.z = q_left[3]
        pose2left.pose.orientation.w = q_left[0]

        pose2left=insertdataPos(pose2left, q_left)

        return pose2left

    else:
        pose=insertdataPos(data, q_left)
        return pose
        
def angle_obj(baseTtarget, targTobj):
    baseRtarget_transp=baseTtarget[0:3, 0:3]
    t_base=baseRtarget_transp.dot(targTobj[0:3, 3])

    x=t_base[0]
    y=t_base[1]
    theta=math.atan2(y, x)

    return theta

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

  return resultTraj


def actionblock(Pose2reach, velocity):
    controller.set_target(Pose2reach)
    #controller.approach_block_straightaway(Pose2reach)
    controller.set_velocity(velocity)
    resultTraj=controller.trajectory_status()
    controller._go_to_moveit_pose_goal(Pose2reach)

    return resultTraj

def homogeneuos_matrix(pos, increment_z):
    q0=pos.pose.orientation.w
    q1=pos.pose.orientation.x
    q2=pos.pose.orientation.y
    q3=pos.pose.orientation.z
    homogeneuos=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2), pos.pose.position.x], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1), pos.pose.position.y],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2, pos.pose.position.z],
        [0, 0, 0, 1]
    ])

    increment_matrix=np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, increment_z],
        [0, 0, 0, 1],
    ])

    final=homogeneuos.dot(increment_matrix)    
    pos.pose.position.x=final[0][3]
    pos.pose.position.y=final[1][3]
    pos.pose.position.z=final[2][3]
    return pos



def learningCallback(opt_learn):
    
    if opt_learn:
        listener.waitForTransform("/edo_base_link", "/handeye_target", rospy.Time(0), rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("/edo_base_link", "/handeye_target", rospy.Time(0))
        base2target=tomatrix(trans, rot)

        off_pos={}
        result=True
        learn_position=Bool()
        k=0
        calib_phase=4

        for i in range(calib_phase):  
            off_pos[i]=np.eye(4, dtype=float)

        off_pos[0][0:3, 3]=np.array([-0.0, -0.4, -0.1], dtype=float)
        off_pos[1][0:3, 3]=np.array([-0.03, -0.4, -0.06], dtype=float)
        off_pos[2][0:3, 3]=np.array([0.03, -0.4, -0.06], dtype=float)
        off_pos[3][0:3, 3]=np.array([0.0, -0.35, -0.02], dtype=float)

        while result and k<calib_phase:

            try:
                pos=looktower(base2target, off_pos[k])
                result=action(pos, 0.3)
                learn_position=True
                learning.publish(learn_position)
                k+=1
                rospy.sleep(2)

            except:
                raise Exception("Solver failed")

        
        else:
            learn_position=False
            #learning.publish(learn_position)
            if not result:
                rospy.loginfo("Cannot execute the %d-th movement", k)

    else:
        learn_position=False
        #learning.publish(learn_position)


def align_block(req):
    listener.waitForTransform("/world", "/edo_gripper_link_ee", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/edo_base_link", "/handeye_target", rospy.Time(0))
    (trans2,rot2) = listener.lookupTransform("/handeye_target", "/edo_gripper_link_ee", rospy.Time(0))
    bTee = tomatrix(trans, rot)

    degree=np.array([
        0,
        0,
        0.70710678118,
        0.70710678118])

    unit=np.array([0, 0, 0, 1])
    T_offset = tomatrix_offset(-0.08, degree)
    T = bTee.dot(T_offset)

    #js_des = np.array([0.0, 0.0, 0.0, 1.57, 0.0, 0.0])
    #controller.go_to_joint(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])
    
    targetPose = insertdataMatrix(T)

    result=action(targetPose, 0.03)
    
    return {'execution':result}

def block_offset(desired_offset_y):
    offset_pose = Pose()
    offset_pose.position.y = desired_offset_y
    offset_pose.position.z = -0.3

    offset_pose.orientation.w = 1

    return offset_pose

def toHomogeneousMatrix(pose):

    matrix=np.eye(4, dtype=float)

    if isinstance(pose, numpy.ndarray):  
        if pose.shape[0] == 3:
            q0 = 1.0
            q1 = 0.0
            q2 = 0.0
            q3 = 0.0

            matrix[0][3] = pose[0]
            matrix[1][3] = pose[1]
            matrix[2][3] = pose[2]

        else:
            q = get_quaternions(pose)


    else:
        q0 = pose.orientation.w 
        q1 = pose.orientation.x
        q2 = pose.orientation.y
        q3 = pose.orientation.z

        q = np.ones(4)
        q = [q0, q1, q2, q3]
        norm = np.linalg.norm(q)

        q0 = q0 / norm
        q1 = q1 / norm
        q2 = q2 / norm
        q3 = q3 / norm

        matrix[0][3] = pose.position.x
        matrix[1][3] = pose.position.y
        matrix[2][3] = pose.position.z

    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    
    matrix[0:3, 0:3]=rot_matrix

    return matrix

def toPoseMsg(bTee_d):
    target = PoseStamped()
    target.pose.position.x = bTee_d[0][3]
    target.pose.position.y = bTee_d[1][3]
    target.pose.position.z = bTee_d[2][3]

    quat = np.zeros(4)
    quat[0] = 0.5*math.sqrt(bTee_d[0][0]+bTee_d[1][1]+bTee_d[2][2]+1)
    quat[1] = (bTee_d[2][1]-bTee_d[1][2])/(4*quat[0])
    quat[2] = (bTee_d[0][2]-bTee_d[2][0])/(4*quat[0])
    quat[3] = (bTee_d[1][0]-bTee_d[0][1])/(4*quat[0])

    norm = np.linalg.norm(quat)
    quat = quat/ norm

    target = insertdataPos(target, quat)

    return target
    
def toTransformStampedMsg(bTee_d):
    target = TransformStamped()

    if isinstance(bTee_d, numpy.ndarray):
        bTee_d = toPoseMsg(bTee_d)
    
    else:
        pass

    target.header.frame_id = "world" #should be block_1_c
    target.header.stamp = rospy.Time.now()
    target.child_frame_id = "layer_1"
    target.transform.translation = bTee_d.pose.position
    
    target.transform.rotation = bTee_d.pose.orientation

    return target

def toReference(matrix):
    matrix_rb_msgs = ReferenceBlock()
    matrix_pose_msgs = toPoseMsg(matrix)

    matrix_rb_msgs.pose = matrix_pose_msgs

    return matrix_rb_msgs


def posblockCallback(pose_block_b):
    
    y_offset = -0.05
    offset = block_offset(y_offset)

    oTc_d = toHomogeneousMatrix(offset)

    if isinstance(pose_block_b, geometry_msgs.msg._PoseStamped.PoseStamped):
        bTo = toHomogeneousMatrix(pose_block_b.pose)

    elif isinstance(pose_block_b, numpy.ndarray):
        bTo = pose_block_b

    listener.waitForTransform("/camera_color_optical_frame", "/edo_gripper_link_ee", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/camera_color_optical_frame", "/edo_gripper_link_ee", rospy.Time(0))
    cdTee = tomatrix(trans, rot)

    bTc_d = bTo.dot(oTc_d)

    bTee_d = bTc_d.dot(cdTee)
    targetPose = toPoseMsg(bTee_d)

    result=action(targetPose, 0.6)

    if not result: #try with a new pose
        offset = block_offset(0.01)
        bTc_d = bTo.dot(oTc_d)

        bTee_d = bTc_d.dot(cdTee)
        targetPose = toPoseMsg(bTee_d)
        result = action(targetPose, 0.6)

def initialize_topblock(topmost_block): #publish only the upper block found
    global bTo_top

    cTo = toHomogeneousMatrix(topmost_block.pose.pose)

    listener.waitForTransform("/world", "/camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/world", "/camera_color_optical_frame", rospy.Time(0))
    
    bTc = tomatrix(trans, rot)

    bTo = bTc.dot(cTo)
    bTo_top = toTransformStampedMsg(bTo)

    bTo_top.child_frame_id = "block "+ topmost_block.location.position + " " + topmost_block.location.orientation + " " + str(topmost_block.location.layer) 

    br_static.sendTransform(bTo_top)

    first_layer_orient.publish(topmost_block.location.orientation)



    return bTo_top

def posupperblockCallback(topmost_block): #publish only the upper block found
    global bTo_top

    result = istopmostlayer(topmost_block)

    if result:
        cTo = toHomogeneousMatrix(topmost_block.pose.pose)

        listener.waitForTransform("/world", "/camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("/world", "/camera_color_optical_frame", rospy.Time(0))
        
        bTc = tomatrix(trans, rot)

        bTo = bTc.dot(cTo)
        bTo_top = toTransformStampedMsg(bTo)

        bTo_top.child_frame_id = "block "+ topmost_block.location.position + " " + topmost_block.location.orientation + " " + str(topmost_block.location.layer) 
    
        br_static.sendTransform(bTo_top)

    else:
        pass
    
def random_y_vel():
    y_vel = TwistStamped()
    y_vel.twist.linear.y = np.random.uniform(low=-0.3, high=0.3)
    for i in range(6):
        js_des = velController.compute_jv(y_vel)
        controller.go_to_joint(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])

def istopmostlayer(new_layer):
    RFs = listener.getFrameStrings()

    for item in RFs:
        if not item.find('block'):
            topmost_block = item
            cTo = toHomogeneousMatrix(new_layer.pose.pose)

            listener.waitForTransform("/world", "/" + topmost_block, rospy.Time(0), rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform("/world", "/" + topmost_block, rospy.Time(0))

            listener.waitForTransform("/world", "/camera_color_optical_frame", rospy.Time(0), rospy.Duration(4.0))
            (trans2,rot2) = listener.lookupTransform("/world", "/camera_color_optical_frame", rospy.Time(0))
            
            bTc = tomatrix(trans2, rot2)

            bTo = bTc.dot(cTo)
            if bTo[2][3] > trans[2]:
                new_topmost_block = True

            else:
                new_topmost_block = False

            break

        else:
            new_topmost_block = True  #useful for initialization --layer18


    return new_topmost_block    

def FirstLayerPoseCallback(first_layer_srv):
    global already_positioned

    found = first_layer_srv.found
    cTl1 = first_layer_srv.cTlayer1

    if not found.data:
        if already_positioned:
            random_y_vel()
            finished.data = True

        else:
            finished.data = go_to_default_pose(0.15, 0.0, 0.68)
            already_positioned = True
    
    else:
        initialize_topblock(cTl1)
        rospy.loginfo("Initialized the policy")
        

    return FirstLayerPoseResponse(
        finished
    )

def BlockEstimationCallback(block_est_serv):
    global bTo_top, Flag_threshold

    Flag_threshold = False
    location_top = bTo_top.child_frame_id.split()
    position_top = location_top[1]
    orientation_top = location_top[2]
    layer_top = location_top[3]

    position = block_est_serv.block_choice.position
    orientation = block_est_serv.block_choice.orientation
    layer= block_est_serv.block_choice.layer

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

    listener.waitForTransform("/world", bTo_top.child_frame_id, rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/world", bTo_top.child_frame_id, rospy.Time(0))
    bTo_topmost = tomatrix(trans, rot)

    bTo.pose = toPoseMsg(bTo_topmost)
    
    diff_layer = float(geometric_params[4]) - geometric_params[5]
    

    """ Doesn't work :(
    bTo = bTo_top
    height = take_height(bTo)

    print(height)

    rospy.loginfo("After 3 is:")
    rospy.loginfo(bTo_top)

    bTo.pose.pose.position.z = height - (diff_layer*0.015)

    rospy.loginfo("After 3 is:")
    rospy.loginfo(bTo_top)

    print(bTo.pose.pose.position.z)
    """

    q_z = pyquaternion.Quaternion(axis = [0, 1, 0], angle = 0.00)

    n_blocks = 0.0
    sign_block = 1.0
    change_orient = 0.0
    z_blocks = 3.0

    if geometric_params[2] == 'dx' and geometric_params[3] == 'dx':
        if geometric_params[0] == 'sx' and geometric_params[1] == 'sx':
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


    elif geometric_params[2] == 'dx' and geometric_params[3] == 'sx':
        change_orient = 1.0

        if geometric_params[0] == 'sx' and geometric_params[1] == 'sx':
            n_blocks = 1.0
            sign_block = -1.0

            z_blocks = 3.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'dx':
            n_blocks = 3.0
            sign_block = -1.0
            
            z_blocks = 1.0    

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'cx':
            n_blocks = 2.0
            sign_block = -1.0
            
            z_blocks = 2.0    

        elif geometric_params[0] == 'sx' and geometric_params[1] == 'dx':
            n_blocks = 1.0
            sign_block = -1.0

            z_blocks = 1.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'sx':
            n_blocks = 3.0
            sign_block = -1.0

            z_blocks = 3.0

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'dx':
            n_blocks = 2.0
            sign_block = -1.0

            z_blocks = 1.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'cx':
            n_blocks = 3.0
            sign_block = -1.0

            z_blocks = 2.0

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'sx':
            n_blocks = 2.0
            sign_block = -1.0

            z_blocks = 3.0

        elif geometric_params[0] == 'sx' and geometric_params[1] == 'cx':
            n_blocks = 1.0
            sign_block = -1.0

            z_blocks = 2.0
            
        else:
            rospy.logwarn("Provide a right convention's name")
        q_z = pyquaternion.Quaternion(axis = [0, 1, 0], angle = math.pi/2)

    elif geometric_params[2] == 'sx' and geometric_params[3] == 'sx':

        if geometric_params[0] == 'sx' and geometric_params[1] == 'sx':
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

    elif geometric_params[2] == 'sx' and geometric_params[3] == 'dx':
        change_orient = 1.0

        if geometric_params[0] == 'sx' and geometric_params[1] == 'sx':
            n_blocks = 3.0
            sign_block = 1.0

            z_blocks = 1.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'dx':
            n_blocks = 1.0
            sign_block = 1.0

            z_blocks = 3.0    

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'cx':
            n_blocks = 2.0
            sign_block = 1.0 
 
            z_blocks = 2.0    

        elif geometric_params[0] == 'sx' and geometric_params[1] == 'dx':
            n_blocks = 3.0
            sign_block = 1.0

            z_blocks = 3.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'sx':
            n_blocks = 1.0
            sign_block = 1.0

            z_blocks = 1.0

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'dx':
            n_blocks = 2.0
            sign_block = 1.0

            z_blocks = 3.0

        elif geometric_params[0] == 'dx' and geometric_params[1] == 'cx':
            n_blocks = 1.0
            sign_block = 1.0

            z_blocks = 2.0

        elif geometric_params[0] == 'cx' and geometric_params[1] == 'sx':
            n_blocks = 2.0
            sign_block = -1.0

            z_blocks = 1.0

        elif geometric_params[0] == 'sx' and geometric_params[1] == 'cx':
            n_blocks = 3.0
            sign_block = 1.0

            z_blocks = 2.0

        q_z = pyquaternion.Quaternion(axis = [0, 1, 0], angle = math.pi/2)

    else:
        rospy.logwarn("Provide a right convention's name")

    block_dimensions = np.array([0.025, 0.015, 0.075])
    #Make the translation
    p1Tp2_transl = np.array([sign_block*(n_blocks*0.025-0.5*change_orient*0.025), diff_layer*0.015, change_orient*(z_blocks*0.025-0.025/2)])
    rospy.loginfo(p1Tp2_transl)

    p1Tp2_mat = toHomogeneousMatrix(p1Tp2_transl)
    p1Tp2_mat[0:3, 0:3] = quat2mat(q_z)

    rospy.loginfo(p1Tp2_mat)

    bTo = toHomogeneousMatrix(bTo.pose.pose)
    bTo = bTo.dot(p1Tp2_mat)

    bTo_block = toTransformStampedMsg(bTo)
    bTo_block.child_frame_id = "block "+ geometric_params[1] + " " + geometric_params[3] + " " + str(geometric_params[5]) 

    br.sendTransform(bTo_block)

    return bTo

def take_height(matrix):
    return matrix.pose.pose.position.z

def tocTo(bTo):
    listener.waitForTransform("/camera_color_optical_frame", "/edo_base_link", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/camera_color_optical_frame", "/edo_base_link", rospy.Time(0))
    cTb = tomatrix(trans, rot)

    cTo = cTb.dot(bTo)

    return cTo

def quat_molt(q_1, q_2_pose_st):
    q_molt = PoseStamped()
    q_2 = pyquaternion.Quaternion(q_2_pose_st.pose.orientation.w, q_2_pose_st.pose.orientation.x, q_2_pose_st.pose.orientation.y, q_2_pose_st.pose.orientation.z)
    q_molt_quat = q_1*q_2

    q_molt.pose.orientation.w = q_molt_quat[0]
    q_molt.pose.orientation.x = q_molt_quat[1]
    q_molt.pose.orientation.y = q_molt_quat[2]
    q_molt.pose.orientation.z = q_molt_quat[3]

    return q_molt



def posCallback(data):
    
    targetPose=insertdataPos(data, qleft)
    result=action(targetPose, 0.6)

def posPBVSCallback(data):
    q_initial=Quaternion()
    q_initial.x=data.pose.orientation.x
    q_initial.y=data.pose.orientation.y
    q_initial.z=data.pose.orientation.z
    q_initial.w=data.pose.orientation.w
    targetPose=insertdataPos(data, q_initial)
    result=action(targetPose, 0.6)

def lastPoseCallback(last):
    global lastPoseReceived, PoseReceived

    lastPoseReceived = last
    PoseReceived = True

def forceCallback(force):
    global Flag, PoseReceived, lastPoseReceived

    if not Flag:

        if force.data>0.35:
            Flag = True
            retract_position = -0.03        
            velocity = TwistStamped()
            velocity.twist.linear.z = retract_position
            
            for i in range(5):
                js_des = velController.compute_jv(velocity)
                controller.go_to_joint(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])
        
        else:
            if PoseReceived: #toavoid to perform such operations when no detection
                listener.waitForTransform("/camera_color_optical_frame", "/handeye_target", rospy.Time(0), rospy.Duration(4.0))
                (cTo_trans,rot) = listener.lookupTransform("/camera_color_optical_frame", "/handeye_target", rospy.Time(0))
                
                if lastPoseReceived.position.z - cTo_trans[2] > 0.02: #0.10: #finished
                    PoseReceived = False
                    
                else:
                    pass

            else:
                pass

    else:
        Flag=False
        pass

def PBVSCallback(velocity):
    global Flag   #integration with force sensor

    if not Flag:
        js_des = velController.compute_jv(velocity)
        controller.go_to_joint(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])

    else:
        pass
    """
    norm_q = np.linalg.norm(js_des-js)
    
    if norm_q>0.2:
        controller.go_to_joint(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])
    
    else:
        controller.go_to_joint2(js_des[0], js_des[1], js_des[2], js_des[3], js_des[4], js_des[5])

    rospy.loginfo("Concluded")
    """




def generalCallback(data):
    global ps
    global J
    global frequency
    global k

    k = k+1
    rospy.loginfo(k)
    # Compute R
    actual_pose=controller.get_current_pose()
    listener.waitForTransform("/world", "/edo_gripper_link_ee", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/edo_base_link", "/edo_gripper_link_ee", rospy.Time(0))
    q0=actual_pose.pose.orientation.w
    q1=actual_pose.pose.orientation.x
    q2=actual_pose.pose.orientation.y
    q3=actual_pose.pose.orientation.z
    rot_matrix=np.array([ 
        [q0**2+q1**2-q2**2-q3**2, 2*(q1*q2-q0*q3), 2*(q1*q3+q0*q2)], 
        [2*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2*(q2*q3-q0*q1)],
        [2*(q1*q3-q0*q2), 2*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
    ])

    p_dot=np.array([
        [data.twist.linear.x], [data.twist.linear.y], [data.twist.linear.z]
    ])

    omega=np.array([
        [data.twist.angular.x], [data.twist.angular.y], [data.twist.angular.z]
    ])

    vel_ee=np.array([
        [p_dot[0][0]], [p_dot[1][0]], [p_dot[2][0]], [omega[0][0]], [omega[1][0]], [omega[2][0]]
    ])

    #v_b=v_ee*R
    p_dot_base=rot_matrix.dot(p_dot)
    omega_base=rot_matrix.dot(omega)
    vel=np.array([
        [p_dot_base[0][0]], [p_dot_base[1][0]], [p_dot_base[2][0]], [omega_base[0][0]], [omega_base[1][0]], [omega_base[2][0]]
    ])

    J=controller.jacobian()
    inv_J=np.linalg.inv(J)
    joints_vel=inv_J.dot(vel)  #[rad/s]

    confirm=J.dot(joints_vel)

    rank_J=np.linalg.matrix_rank(J, tol=0.4)

    if rank_J==0:
        rospy.loginfo("\n \n \n !!!!!!Detected singularity!!!!!!!! \n \n \n ")

    
    #rospy.loginfo(confirm)

    #result=controller.traj_ended(js)
    incremenal_pos=joints_vel/frequency
    norm_pos=np.linalg.norm(incremenal_pos)

    frequency=updatefrequency(norm_pos, frequency)
    incremenal_pos=joints_vel/4

    js=controller.get_current_joint_values()

    for i in range(6):
        if (js[i]+incremenal_pos[i][0])>max[i] or (js[i]+incremenal_pos[i][0])<min[i]:
            sign=math.copysign(1, js[i])
            js[i]=sign*(abs(js[i])-0.1*0.01745329251)
              
        else:
            js[i]=js[i]+incremenal_pos[i][0]

    if norm_pos>0.002:
        controller.go_to_joint(js[0], js[1], js[2], js[3], js[4], js[5])
    
    else:
        controller.go_to_joint2(js[0], js[1], js[2], js[3], js[4], js[5])

def init_Jenga():
    #play = input('Do you want to play Jenga? [y/n]:')

    #if play=="y" or  play=="y" or  play=="yes" or  play=="Yes" or  play=="YES":
    serv_FirstLayerPose = rospy.Service('FirstLayerPose', FirstLayerPose, FirstLayerPoseCallback)
    serv_BlockEstimation = rospy.Service('/BlockEstimation', BlockEstimation, BlockEstimationCallback)
    print("Great! Let's begin")

    #elif play=="n" or  play=="N" or  play=="no" or  play=="No" or  play=="NO":
     #   print("I am so sorry")

    #else:
        #print("Please enter a correct answer")


Flag=False
Flag_vel=False
ps=False
frequency=3

if __name__ == "__main__":
    simplcontrol=rospy.init_node("control_manager", anonymous=True)
    rospy.sleep(3)
    
    q1=pyquaternion.Quaternion(axis=[0,0,1], angle=-math.pi/4)  #q1=pyquaternion.Quaternion(axis=[0,0,1], angle=math.pi/4)
    q2=pyquaternion.Quaternion(axis=[0,1,0], angle=math.pi/2) 
    qleft=q2 #q1*q2

    listener = tf.TransformListener()
    br_static = tf2_ros.StaticTransformBroadcaster()
    br = tf2_ros.TransformBroadcaster()

    J=controller.jacobian()
    (min, max)=controller.get_current_joint_limits()
    
    #s=rospy.Service('Pose_cao_initializer', InitialGuess, initial_guest)
    positioning_upper=rospy.Subscriber('object_odometry/topmost_layer', ReferenceBlock, posupperblockCallback)
    positioning=rospy.Subscriber('object_odometry/blockposition', PoseStamped, posblockCallback)
    sub=rospy.Subscriber('/cartesian',PoseStamped,posCallback)

    calibration=rospy.Subscriber('learning_calibration', Bool, learningCallback)
    initial_guest=rospy.Subscriber('initialGuestPos',PoseStamped, posPBVSCallback)
    #subPBVS=rospy.Subscriber('/servo_server/delta_twist_cmds',TwistStamped, generalCallback, queue_size=1)
    subPBVS=rospy.Subscriber('/servo_server/delta_twist_cmds',TwistStamped, PBVSCallback, queue_size=1)
    subForce=rospy.Subscriber('/appliedForce',Float32, forceCallback)

    learning = rospy.Publisher('learning_phase', Bool, queue_size=1)
    counting = rospy.Publisher('couting', Int32, queue_size=1)
    first_layer_orient = rospy.Publisher('/init_orientation', String, queue_size=1)   
    last_pose=rospy.Subscriber('/lastPose',Pose, lastPoseCallback)

    init_Jenga()

    rospy.spin()
