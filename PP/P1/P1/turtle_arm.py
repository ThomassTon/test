#! /usr/bin/python

import math

##-----------------------------------------------------------------
##-- Definition of the turtlebot's kinematic
##-----------------------------------------------------------------
def turtle_arm_dh():
  # define dictionary containing dh parameters

  #dh = [0       0.0405+0.036    0       pi/2    1;
  #      1.41    0               0.1499  0       1;
  #      0.1608  0               0.15    0       1;
  #      0       0               0.14    0       1];
  dh = [{'theta': 0,      'd': 0.0405+0.036, 'a': 0,      'alpha': math.pi/2, 'rho': 1},
        {'theta': 1.41,   'd': 0,            'a': 0.1499, 'alpha': 0,         'rho': 1},
        {'theta': 0.1608, 'd': 0,            'a': 0.15,   'alpha': 0,         'rho': 1},
        {'theta': 0,      'd': 0,            'a': 0.14,   'alpha': 0,         'rho': 1}]

  return dh

