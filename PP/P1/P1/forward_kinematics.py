#! /usr/bin/python

import math
import numpy as np

from turtlebot3_ik_solver_exercise.ik_helper import compute_transformation_dh

def forward_kinematics_dh(dh, q):
  '''
  The method forward_kinematics_dh computes the transformation
  matrices, which describe the position and orientation of the
  joints of the manipulator with regard to the base coordinate
  system.
  Input values are the list containing the DH-Parameter dictionary of
  all links of the kinematic chain and the joint variables.

  INPUT
    dh                     List containing the dictionaries with four DH-Parameter and one joint type
    dh[i]["theta"]  [1x1]  DH-Parameter theta of link i+1
    dh[i]["d"]      [1x1]  DH-Parameter d of link i+1
    dh[i]["a"]      [1x1]  DH-Parameter a of link i+1
    dh[i]["alpha"]  [1x1]  DH-Parameter alpha of link i+1
    dh[i]["rho"]    [1x1]  Joint type (0 = prismatic, 1 = revolute)
    q                      Array of joint N variables in [rad] or [m] according to the joint type

  OUTPUT
    TM               List of all transformation matrices
    TM[i]     [4x4]  Transformation matrix of the (i+1)-th robot link with regard to the base
  '''

  # initialize return value
  TM = [np.eye(4)] * len(dh)
  # | Implement your code here |
  # v                          v
  length = len(dh)
  for i in range(length):
    TM[i] = compute_transformation_dh(dh[i], q[i])


  # ^                          ^
  # | -------- End ----------- |

  return TM
