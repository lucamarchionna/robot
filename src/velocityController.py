#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
import tf
import scipy
from edocontroller import EdoFactory, REAL_ROBOT
from edo_core_msgs.msg import JointInit, CollisionThreshold, MovementCommand, CartesianPose, Point
import numpy as np
from edo_core_msgs.msg import JointControlArray, JointControl, JointStateArray
from std_msgs.msg import String, Float32, Int32, Float64, Bool
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped

global actual_time, finished_time

class VelocityController:

    def __init__(self, execution_type):
        self.edo = EdoFactory().create_robot_model(REAL_ROBOT)
        self.joints_number = 6
        (self.q_min, self.q_max) = self.edo.get_current_joint_limits()
        self.execution = execution_type
        self.jprint = False
        
        

    def compute_jv(self, v):
        js = self.edo.get_current_joint_values()
        joints_number = self.joints_number+1 #include end-effector
        actual_pose = self.current_pose
        quat = self.get_quaternions() #order: x,y,z,w
        trasl = self.get_position()
        (bRee, bTee) = self.tomatrix(trasl, quat)

        vel = self.toarray(v)
        vel_base = self.toBF(vel, bRee)
        vel_base = self.velocity_rectifier(vel_base)

        J = self.edo.jacobian()

        inv_J = np.linalg.inv(J)
        inv_J = scipy.linalg.inv(J, overwrite_a=False, check_finite=False)

        q_dot = inv_J.dot(vel_base)  #[rad/s]
        dt = 0.1
        incremenal_q = q_dot*dt        
        
        q_des = self.joint_limits_rectifier(incremenal_q)

        #q_des = self.singularity_handling(q_des)

        if self.execution == "REAL" :
            JCAmsg = self.toJCA(q_des)
            return JCAmsg
            

        elif self.execution == "SIM":
            return q_des

        else:
            rospy.logwarn("You need to specify the execution type: SIM or REAL", )
     


    def angle_obj(self, baseTtarget, targTobj):
        baseRtarget_transp = baseTtarget[0:3, 0:3]
        t_base = baseRtarget_transp.dot(targTobj[0:3, 3])

        x = t_base[0]
        y = t_base[1]
        theta = math.atan2(y, x)

        return theta
     

    def current_pose(self):
        return self.edo.get_current_pose()

    
    def get_position(self):
        actual = self.current_pose()
        pos = np.ones(3)
        pos[0] = actual.pose.position.x
        pos[1] = actual.pose.position.y
        pos[2] = actual.pose.position.z

        return pos


    def get_quaternions(self):
        
        actual = self.current_pose()
        q = np.ones(4)
        q[3] = actual.pose.orientation.w
        q[0] = actual.pose.orientation.x
        q[1] = actual.pose.orientation.y
        q[2] = actual.pose.orientation.z

        norm = np.linalg.norm(q)

        q = q / norm

        return q

    def joint_limits_rectifier(self, incremenal_pos):
        js=self.edo.get_current_joint_values()
        final_js = np.zeros(self.joints_number)
        tol = 0.5
        for i in range(6):
            if (js[i]+incremenal_pos[i][0])>self.q_max[i] or (js[i]+incremenal_pos[i][0])<self.q_min[i]:
                sign=math.copysign(1, js[i])
                js[i]=sign*(abs(js[i])-0.1*0.01745329251)
                    
            else:
                
                js[i]=js[i]+incremenal_pos[i][0]

        # rospy.loginfo(js)
        # rospy.loginfo("\n")
        return js


    def outside_singularities(self, joint, degrees):
        sign = math.copysign(1, joint)
        new_joint_value = sign*(abs(joint) - degrees*0.01745329251)

        return new_joint_value

    def velocity_rectifier(self, vel):
        max_vel = 0.02

        for i in range(6):
            if abs(vel[i]) > max_vel:
                sign = math.copysign(1, vel[i])
                vel[i] = sign*max_vel
            else:
                pass
        
        return vel

    def singularity_handling(self, q_des):
        J = self.edo.jacobian()
        rank_J = np.linalg.matrix_rank(J)
        q = self.edo.get_current_joint_values()
        
        if rank_J < 6:
            rospy.loginfo("Detected a singularity in the actual position : %d" ,rank_J)
            singularity = True # if a singularity is present move only the first joint
            q[2] = self.outside_singularities(q[2], 3)
            q[4] = q[4] + 2*0.01745329251


            q_des = q
        
        else:
            singularity = False

        return q_des


    def waypoints(self, joint_values, points):
        js_waypoints = np.zeros((points, 6), dtype=float)

        for k in range(self.joints_number):
            i = 0
            for i in range(points):
                js_waypoints[i][k] = joint_values[k]*(i+1)/points
        
        return js_waypoints

    
    def toBF(self, vel, rot_matrix):
        p_dot = vel[0:3]
        omega = vel[3:]
        p_dot_base = rot_matrix.dot(p_dot)
        omega_base = rot_matrix.dot(omega)

        vel = np.array([
        [p_dot_base[0][0]], [p_dot_base[1][0]], [p_dot_base[2][0]], 
        [omega_base[0][0]], [omega_base[1][0]], [omega_base[2][0]]
        ],  dtype=float)

        return vel


    def toarray(self, velTS):  #TS: Twist stamped
        p_dot = np.array([
            [velTS.twist.linear.x], [velTS.twist.linear.y], [velTS.twist.linear.z]
        ],  dtype=float)

        omega = np.array([
            [velTS.twist.angular.x], [velTS.twist.angular.y], [velTS.twist.angular.z]
        ],  dtype=float)

        v_array = np.array([
            [p_dot[0][0]], [p_dot[1][0]], [p_dot[2][0]],
            [omega[0][0]], [omega[1][0]], [omega[2][0]]],  dtype=float)

        return v_array


    def toJCA(self, final_js):
        joints = JointControlArray()
        joints.size = 7
        joints.joints = [JointControl(57.2958*(final_js[i]) , 0.0001, 0, 0, 0, 0) for i in range(6)]+[JointControl(0.0, 0, 0, 0, 0, 0)]
        return joints



    def tomatrix(self, transl, quat):
        q0 = quat[3]
        q1 = quat[0]
        q2 = quat[1]
        q3 = quat[2]
        matrix = np.eye(4, dtype=float)

        rot_matrix = np.array([ 
            [q0**2+q1**2-q2**2-q3**2, 2.00*(q1*q2-q0*q3), 2.00*(q1*q3+q0*q2)], 
            [2.00*(q1*q2+q0*q3), q0**2-q1**2+q2**2-q3**2, 2.00*(q2*q3-q0*q1)],
            [2.00*(q1*q3-q0*q2), 2.00*(q2*q3+q0*q1), q0**2-q1**2-q2**2+q3**2]
        ],  dtype=float)

        matrix[0:3, 0:3] = rot_matrix
        matrix[0:3, 3] = transl

        return rot_matrix, matrix

"""section for singularity handling of motion rate controller
    def store_previous_mom(self):
        if k==0:
            J = self.edo.jacobian()
            mom = math.sqrt( numpy.linalg.det(J.dot(J.transpose)) )
            k = k+1

        else:
            J_prev = self.store_previous_J()
            J_actual = self.edo.jacobian()


    def store_previous_J(self):
        J = self.edo.jacobian()
        return J



    def mom_singularity_handling(self):
        J = self.edo.jacobian()
        mom = math.sqrt( numpy.linalg.det(J.dot(J.transpose)) )
"""

if __name__ == "__main__":
    velNode=rospy.init_node("MotionRate_controller", anonymous=True)

    rospy.spin()