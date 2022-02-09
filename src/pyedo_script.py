#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
from geometry_msgs.msg import Pose, Quaternion, Point, PoseStamped
from pyedo import edo
import pyedo



if __name__ == "__main__":
    edo = edo('10.42.0.49')
    edo.init7Axes()
    edo.disengageStd()
    edo.calibAxes()
    edo.moveSingleJoint(1, 50)

