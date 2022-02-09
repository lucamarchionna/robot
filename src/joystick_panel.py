#!/usr/bin/env python3

import rospy
from time import sleep
import subprocess
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped

def insertdata(data):
    twist = TwistStamped()
    twist.twist.linear.x = data[0]
    twist.twist.linear.y = data[1]
    twist.twist.linear.z = data[2]
    twist.twist.angular.x = data[3]
    twist.twist.angular.y = data[4]
    twist.twist.angular.z = data[5]

    return twist

def joystickCallback(data):
    actuated = False
    twist = TwistStamped()
    scaling_factor = 0.03
    
    #position
    # pos_x,y,z orien_x,y,z
    pos_x = -data.axes[0] *scaling_factor
    pos_y = data.axes[1] *scaling_factor
    pos_z = 0

    if data.buttons[4]!=0 or data.buttons[5]!=0:
        if data.buttons[4]!=0:
            sign = -1
        
        else:
            sign = 1

        pos_z = sign * scaling_factor
    
    #orientation
    orien_x = -data.axes[3] *scaling_factor
    orien_y = data.axes[4] *scaling_factor
    orien_z = 0

    if data.axes[5]==-1 or data.axes[2]==-1:
        if data.axes[2]==-1:
            sign = -1
        
        else:
            sign = 1

        orien_z = sign * scaling_factor

    pose = [pos_x, pos_y, pos_z, orien_x, orien_y, orien_z]
    
    for p in pose:
        if p!=0:
            actuated = True
        else:
            pass
    
    if actuated:
        twist = insertdata(pose)
        servo_motor.publish(twist)

    



if __name__=="__main__":

    rospy.init_node("joystick_edo")
    rospy.loginfo("UI for controlling e.DO with a joystick")
    joySub = rospy.Subscriber('/joy', Joy, joystickCallback)
    servo_motor = rospy.Publisher('/servo_server/delta_twist_cmds', TwistStamped, queue_size=1, latch=True)

    rospy.spin()


