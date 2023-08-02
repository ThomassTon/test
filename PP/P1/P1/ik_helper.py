

import math
import numpy as np
from numpy import linalg
from numpy.core.fromnumeric import shape

from turtlebot3_ik_solver_exercise.turtle_arm import turtle_arm_dh

def normalize_angles(angles):
  '''
  Normalized angles in passed array to interval [-pi, pi]

  INPUT:
    angles              (List) Array of angles in [rad]

  OUTPUT
    angles_normalized   (List) Array of normalized angles
  '''

  angles_normalized = angles

  if isinstance(angles, list):
    for i in range(len(angles_normalized)):
      angles_normalized[i] = np.mod(np.mod(angles_normalized[i], 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)
      for angle_normalized in np.nditer(angles_normalized[i], op_flags=['readwrite']):
        if angle_normalized > math.pi:
          angle_normalized -= 2.0 * math.pi
  else:
    angles_normalized = np.mod(np.mod(angles_normalized, 2.0*math.pi) + 2.0*math.pi, 2.0*math.pi)
    for angle_normalized in np.nditer(angles_normalized, op_flags=['readwrite']):
      if angle_normalized > math.pi:
        angle_normalized -= 2.0 * math.pi

  return angles_normalized


def get_tcp_position(TM):
  '''
    The method get_tcp_position extracts the endeffector position
    with regard to the base from the data structure of the
    manipulators transformations resulting form forwardKinematicsDH.

    INPUT
      TM:      [N]    List containing all N transformation matrices
      TM[i]:   [4x4]  Transformation matrix (2D python array)
                      of the (i+1)-th robot link with regard to the base
    OUTPUT
      p:       [3x1]  Endeffector possition with regard to the base
  '''
  # initialize return value
  p = np.zeros((3, 1))
  T = np.eye(4)
  # | Implement your code here |
  # v                          v
  length = len(TM)
  for i in range(length):
    # print("T",i,": \n",TM[i])
    T = T.dot(TM[i])

  for i in range(3):
    p[i, 0] = T[i, 3]

  # print("p: \n", p[0,0]," \n", p[1,0]," \n ", p[2,0])
  # print("T: \n", T)

  # ^                          ^
  # | -------- End ----------- |

  return p


def compute_transformation_dh(dh, q):
  '''
  The function compute_transformation_dh computes, given the list
  dh containing the DH-Parameter as well as the joint value q, the
  local transformation matrix of one robot link.

  INPUT
    dh:            Dictionary containing four DH-Parameter and one joint type\n
    dh["theta"]:   [1x1]  DH-Parameter theta\n
    dh["d"]:       [1x1]  DH-Parameter d\n
    dh["a"]:       [1x1]  DH-Parameter a\n
    dh["alpha"]:   [1x1]  DH-Parameter alpha\n
    dh["rho"]:     [1x1]  Joint type (0 = prismatic, 1 = revolute)\n
    q:             [1x1]  Joint variable in [rad] or [m] according to the joint type

  OUTPUT
    T:             [4x4]  Transformation matrix (2D python array) of the robot link
  '''
  # initialize return value
  T: float = np.eye(4)

  # | Implement your code here |
  # v                          v
  theta = dh["theta"]
  d = dh["d"]
  a = dh["a"]
  alpha = dh["alpha"]
  rho = dh["rho"]

  if rho == 1:
    T[0][0] = np.cos((theta+q))

    T[0][1] = -np.sin((theta+q))*np.cos(alpha)
    T[0][2] = np.sin((theta+q))*np.sin(alpha)
    T[0][3] = a*np.cos(theta+q)

    T[1][0] = np.sin((theta+q))
    T[1][1] = np.cos((theta+q))*np.cos(alpha)
    T[1][2] = -np.cos((theta+q))*np.sin(alpha)
    T[1][3] = a*np.sin(theta+q)

    T[2][1] = np.sin(alpha)
    T[2][2] = np.cos(alpha)
    T[2][3] = d
  else:
    T[0][0] = np.cos(theta)
    T[0][1] = -np.sin(theta)*np.cos(alpha)
    T[0][2] = np.sin(theta)*np.sin(alpha)
    T[0][3] = a*np.cos(theta)

    T[1][0] = np.sin(theta)
    T[1][1] = np.cos(theta)*np.cos(alpha)
    T[1][2] = -np.cos(theta)*np.sin(alpha)
    T[1][3] = a*np.sin(theta)

    T[2][1] = np.sin(alpha)
    T[2][2] = np.cos(alpha)
    T[2][3] = d+q


  # ^                          ^
  # | -------- End ----------- |
  return T
