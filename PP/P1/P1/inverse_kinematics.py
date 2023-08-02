#! /usr/bin/python

import math
import numpy as np


def inverse_kinematics_tb(request_position, py, l):
  '''
  The method inverse_kinematics_tb computes the joint configurations
  suitable to reach the requested position and orientation in
  both elbow configurations.

  Input values are the endeffector position, the endeffector
  orientation with the yaw rotation about the z_0-axis and
  pitch angle between the z_0-axis and the endeffector direction
  described by x_n-axis.

  INPUT
    request_position       [3x1]  Requested endeffector position as [x, y, z]'
    py                            Dictionary containing the requested pitch and yaw
    py["pitch"]                   Requestet pitch
    py["yaw"]                     Requestet yaw
    l                             List containing N elements according to the link lengths

  OUTPUT
    qs                            List containing two joint configuration arrays of shape [1xN]
                                  or empty [] if no solution can be found
  '''
  # initialize return value
  qs = []

  # | Implement your code here |
  # v                          v

  Rot_z = np.zeros((3, 3))
  Rot_z[0, 0] = np.cos(-py["yaw"])
  Rot_z[0, 1] = -np.sin(-py["yaw"])
  Rot_z[1, 0] = np.sin(-py["yaw"])
  Rot_z[1, 1] = np.cos(-py["yaw"])
  Rot_z[2, 2] = 1
  p = py["pitch"]
  p0 = Rot_z.dot(request_position)
  x4 = p0[0]
  z4 = p0[2]
  if (((-math.pi/2)<p<0)&(x4>0))|(((math.pi/2)>p>0)&(x4<0)):
    z3 = z4 - np.cos(p)*l[3]
    x3 = x4 - np.sin(p)*l[3]
  # elif(((-math.pi)<p<(-math.pi/2))&(x4>0))|(((math.pi)>p>(math.pi/2))&(x4<0)):
  #   z3 = z4 - np.sin(p-math.pi/2)*l[3]
  #   x3 = x4 - np.cos(p-math.pi/2)*l[3]
  else:     
    z3 = z4 + np.sin(p-math.pi/2)*l[3]
    x3 = x4 - np.cos(p-math.pi/2)*l[3]


  c_q = np.square(z3 - l[0]) + np.square(x3)
  # print("c_q: ", c_q)
  cos_q3 = (c_q - np.square(l[2]) - np.square(l[1])) / (2 * l[2] * l[1])
  # print("cos_q3: ", cos_q3)

  sin_q3 = np.zeros((2, 1))
  sin_q3[0, 0] = np.sqrt(1 - np.square(cos_q3))
  sin_q3[1, 0] = -np.sqrt(1 - np.square(cos_q3))

  q3 = np.zeros((2, 1))
  q3[0, 0] = math.atan2(sin_q3[0, 0], cos_q3)
  q3[1, 0] = math.atan2(sin_q3[1, 0], cos_q3)
  # print("q3_1: ", q3[0, 0] / math.pi  * 180, "q3_2", q3[1, 0] / math.pi  * 180)

  beta = np.zeros((2, 1))
  beta[0, 0] = math.atan2(l[2] * sin_q3[0, 0], l[1] + l[2] * cos_q3)
  beta[1, 0] = math.atan2(l[2] * sin_q3[1, 0], l[1] + l[2] * cos_q3)

  # print("beta_1: ", beta[0, 0] / math.pi  * 180, "beta_2: ", beta[1, 0] / math.pi  * 180)

  fai = math.atan2(z3-l[0], x3)
  # print("fai: ", fai / math.pi  * 180)

  q2 = np.zeros((2, 1))
  # if((z3-l[0])>0):
  q2[0, 0] = fai - beta[0, 0] - math.pi  / 2
  q2[1, 0] = fai - beta[1, 0] - math.pi  / 2
  # else :
  #   q2[0, 0] = fai + beta[0, 0] - math.pi  / 2
  #   q2[1, 0] = fai + beta[1, 0] - math.pi  / 2
  # # print("q2_1: ", q2[0, 0] / math.pi  * 180, "q2_2", q2[1, 0] / math.pi  * 180)

  q4 = np.zeros((2, 1))
  q4[0, 0] = -py["pitch"] - q2[0, 0] - q3[0, 0]
  q4[1, 0] = -py["pitch"] - q2[1, 0] - q3[1, 0]

  # print("q4_1: ", q4[0, 0] / math.pi  * 180, "q4_2", q4[1, 0] / math.pi  * 180)

  q1 = py["yaw"]
  qs = [np.zeros((1, 4))]*2
  qs[0] = np.asarray((q1, q2[0, 0], q3[0, 0], q4[0, 0]))
  qs[1] = np.asarray([q1, q2[1, 0], q3[1, 0], q4[1, 0]])

  # ^                          ^
  # | -------- End ----------- |

  return qs
