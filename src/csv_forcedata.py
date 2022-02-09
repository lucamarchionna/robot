#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
import tf
import numpy as np
import csv
from std_msgs.msg import String, Float32, Int32, Float64

def writerCallback(force):
    global writer
    value=round(force.data, 3)
    mydict=[{'Layer p': value}]
    if value>0.00 :
        writer.writerows(mydict) 

    

if __name__ == "__main__":
    measurement=rospy.init_node("measurement", anonymous=True)
    filename = "measurements.csv"
     
    global writer, fields
    fields = ['Layer p'] 
    with open(filename, 'w') as csvfile:
        writer=csv.DictWriter(csvfile, fieldnames = fields) 
        writer.writeheader() 
        subForce=rospy.Subscriber('/appliedForce',Float32,writerCallback)
        rospy.spin()

    