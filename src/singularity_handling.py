#!/usr/bin/env python3

import math
import numpy
import pyquaternion 
import time
import rospy
import tf
import numpy as np

class motionrate:

  def __init__(self):
    self.jacobian = np.array([
    [4, 0],
    [0, 21]
    ])
    self.begin = False

  def rate_motion_singularity(self):
    J_prev = self.get_previous_J()
    J_actual = self.compute_jacobian()

    mom = self.compute_derivative_mom(J_actual, J_prev)
   

  def compute_derivative_mom(self, J_actual, J_prev):
    mom = self.compute_mom(J_actual)
    dJ_dqk = 6 #make here the partial derivative function
    print(dJ_dqk)
    dmom_dqk = mom * np.trace(dJ_dqk*J_actual)


  def compute_mom(self, jac):
    mom = math.sqrt(np.linalg.det(jac.dot(np.transpose(jac))))
    return mom

  def get_previous_J(self):
    if not self.begin:
      self.begin = True
      J = self.jacobian 
      
    else:
      J = self.compute_jacobian()
      
    return J

  def compute_jacobian(self):
    jacobian = np.array([
    [4, 20],
    [0, 56]
    ])

    return jacobian




if __name__ == "__main__":
  velController = motionrate()
  velController.rate_motion_singularity()