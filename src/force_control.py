#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32

if __name__ == "__main__":

    rospy.init_node("ForceData")
    rospy.sleep(3)

    force = rospy.Publisher('/sensorData', Float32,queue_size=2)

    rate=rospy.Rate(0.5)
    count=Float32()
    count.data=0
    while not rospy.is_shutdown():

        count.data=count.data+0.01
        force.publish(count)
        rate.sleep()
        
    rospy.spin()

   
    



