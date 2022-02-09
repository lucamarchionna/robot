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

controller = EdoFactory().create_robot_model(REAL_ROBOT)
Flag=False
ps=False
frequency=1
k=Int32
k=0

def to_quater(cartPose):
    quat=pyquaternion.Quaternion()
    quat[0]=cartPose.pose.orientation.w
    quat[1]=cartPose.pose.orientation.x
    quat[2]=cartPose.pose.orientation.y
    quat[3]=cartPose.pose.orientation.z
    
    return quat



def velCallback(data):
   
    global Flag_vel
    global J
    global frequency
    
    joints=JointControlArray()
    joints.size=7

    # Compute R
    listener.waitForTransform("/world", "/edo_gripper_link_ee", rospy.Time(0), rospy.Duration(4.0))
    (trans,rot) = listener.lookupTransform("/edo_base_link", "/edo_gripper_link_ee", rospy.Time(0))
    q0=qleft[1]
    q1=qleft[2]
    q2=qleft[3]
    q3=qleft[0]

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


"""
def disable_collision(data):
    # before disabling edo_algorithms we need to "disable" collision checking on joints and raspberry
    msg_dis_coll_js = JointInit()
    msg_dis_coll_js.mode = 3
    msg_dis_coll_js.joints_mask = 63
    msg_dis_coll_js.reduction_factor = 10.0

    msg_dis_coll_js_m = JointInit()
    msg_dis_coll_js_m.mode = 3
    msg_dis_coll_js_m.joints_mask = 127
    msg_dis_coll_js_m.reduction_factor = 10.0

    msg_dis_coll_algo = CollisionThreshold()
    msg_dis_coll_algo.joints_mask = 127
    msg_dis_coll_algo.threshold = 10.0

    collision.publish(msg_dis_coll_js)
    algo_col_thr_pub.publish(msg_dis_coll_algo)
    rospy.loginfo("Done collision")

"""
Flag=False
Flag_vel=False
ps=False
frequency=3

def cartesianPose(add_value):
    init1=MovementCommand()
    init1.move_command=80
    init2=MovementCommand()
    init2.move_command=67

    command=MovementCommand()
    command.move_command=77
    command.move_type=76
    command.ovr=50
    command.delay=0
    command.remote_tool=0
    command.cartesian_linear_speed=0.0

    actual_pose=controller.get_current_pose()
    q=to_quater(actual_pose)
    u = q.axis
    rospy.loginfo(u)

    data=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.4299999475479126, 0.0, 0.0, 0.0]
    pose=Point()
    pose.data_type=88
    pose.cartesian_data.x=actual_pose.pose.position.x
    pose.cartesian_data.y=actual_pose.pose.position.y
    pose.cartesian_data.z=actual_pose.pose.position.z+add_value.data
    pose.cartesian_data.a=0
    pose.cartesian_data.e=0
    pose.cartesian_data.r=0
    pose.cartesian_data.config_flags="S E "
    pose.joints_mask=127
    
    pose.joints_data=data
    command.target=pose


    cartesian.publish(command)













if __name__ == "__main__":
    simplcontrol=rospy.init_node("vel_controller", anonymous=True)
    rospy.sleep(4)
    
    q2=pyquaternion.Quaternion(axis=[0,1,0], angle=-math.pi/2)
    
    qleft=q2
    rospy.loginfo(qleft)


    listener = tf.TransformListener()

    subPBVS=rospy.Subscriber('/servo_server/delta_twist_cmds',TwistStamped, velCallback)

    algo=rospy.Publisher('/algo_jnt_ctrl', JointControlArray, queue_size=frequency, latch=True)
    collision=rospy.Publisher('/bridge_init', JointInit, queue_size=10, latch=True)
    algo_col_thr_pub = rospy.Publisher('/algo_coll_thr', CollisionThreshold, queue_size=1, latch=True)
    #collision_sub=sub=rospy.Subscriber('coll',PoseStamped, disable_collision)

    cartesian=rospy.Publisher('/machine_move', MovementCommand, queue_size=1, latch=True)

    collision_sub=sub=rospy.Subscriber('prova_cart', Float32, cartesianPose)
    

    rospy.spin()