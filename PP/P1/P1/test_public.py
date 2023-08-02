#!/usr/bin/env python
PKG = 'turtlebot3_ik_solver_exercise'

import sys
import unittest

import math
import numpy as np
from numpy import linalg

from turtlebot3_ik_solver_exercise.ik_helper import *
from turtlebot3_ik_solver_exercise.forward_kinematics import *
from turtlebot3_ik_solver_exercise.inverse_kinematics import *
from turtlebot3_ik_solver_exercise.mat_nr import *

## A sample python unit test
class TestPublic(unittest.TestCase):
    def setUp(self):
        self.testPassed = True
        self.testName = ""
        self.pointsSet = False
        self.reachedPoints = 0.0        

    def tearDown(self):
        if self.testPassed:
            print("\n\t==> Test [" + self.testName + "] Passed!")
        else:
            print("\n\t==> Test [" + self.testName + "] (partially) Failed! Please see test output above for more information.")

        if self.pointsSet:
            print("\t==> Test [" + self.testName + "] Points: " + str(self.reachedPoints))

        assert self.testPassed, ("\n\t==> Test [" + self.testName + "] (partially) Failed! Please see test output above for more information.")
    
    ## test get tcp position
    def test_1_get_tcp_position(self):
        self.testName = "test_1_get_tcp_position"
        self.pointsSet = True
        ## define a list with transformation matrices
        TM = [np.matrix('2 3 4 5; 4 6 8 10; 6 9 12 15; 0 0 0 1')]
        expected = np.matrix('5; 10; 15')
        position_error = linalg.norm(expected - get_tcp_position(TM))
        try: 
            self.assertAlmostEqual(position_error, 0.0, 6)
            print("\n\t---\n\tTest [" + self.testName + "] ok:\n\tReturned position correct" + "\n\t---")
            self.reachedPoints += 1.0
        except AssertionError as e: 
            print("\n\t---\n\tTest [" + self.testName + "] fails:\n\tReturned position deviates - error: " + str(position_error) + "\n\t---")
            self.testPassed = False

    ## test forward kinematics
    def test_2_forward_kinematics(self):
        self.testName = "test_2_forward_kinematics"
        self.pointsSet = True
        
        dh = [{'theta': math.pi/2, 'd': 0, 'a': 1, 'alpha': 0, 'rho': 1},
              {'theta': 0,         'd': 0, 'a': 1, 'alpha': 0, 'rho': 1},
              {'theta': 0,         'd': 0, 'a': 0, 'alpha': 0, 'rho': 0}]
        q = np.array([-math.pi/2, math.pi/2, 1])

        ## 1st test
        expected_tcp_position = np.matrix('1; 1; 1')
        TM = forward_kinematics_dh(dh, q)
        position_error = linalg.norm(expected_tcp_position - get_tcp_position(TM))
        try: 
            self.assertAlmostEqual(position_error, 0.0, 6)
            print("\n\t---\n\tTest [" + self.testName + "] 1st test ok:\n\tReturned position correct" + "\n\t---")
            self.reachedPoints += 1.0
        except AssertionError as e:
            print("\n\t---\n\tTest [" + self.testName + "] 1st test fails:\n\tReturned position deviates - error: " + str(position_error) + "\n\t---")
            self.testPassed = False

        ## 2nd test
        dh[2]["d"] = 1
        q = np.array([-math.pi/2, math.pi/2, 0])
        expected_tcp_position = np.matrix('1; 1; 1')
        TM = forward_kinematics_dh(dh, q)
        position_error = linalg.norm(expected_tcp_position - get_tcp_position(TM))
        try: 
            self.assertAlmostEqual(position_error, 0.0, 6)
            print("\n\t---\n\tTest [" + self.testName + "] 2nd test ok:\n\tReturned position correct" + "\n\t---")
            self.reachedPoints += 1.0
        except AssertionError as e:
            print("\n\t---\n\tTest [" + self.testName + "] 2nd test fails:\n\tReturned position deviates - error: " + str(position_error) + "\n\t---")
            self.testPassed = False
        
        ## 3rd test
        dh = turtle_arm_dh()
        q = np.array([0, 0, 0, 0])
        expected_tcp_position = np.matrix('2.39985710725893e-02; 2.68177016862059e-17; 5.14466305140020e-01')
        TM = forward_kinematics_dh(dh, q)
        position_error = linalg.norm(expected_tcp_position - get_tcp_position(TM))
        try: 
            self.assertAlmostEqual(position_error, 0.0, 6)
            print("\n\t---\n\tTest [" + self.testName + "] 3rd test ok:\n\tReturned position correct" + "\n\t---")
            self.reachedPoints += 1.0
        except AssertionError as e:
            print("\n\t---\n\tTest [" + self.testName + "] 3rd test fails:\n\tReturned position deviates - error: " + str(position_error) + "\n\t---")
            self.testPassed = False

    ## test inverse kinematics
    def test_3_inverse_kinematics(self):
        self.testName = "test_3_inverse_kinematics"
        dh = turtle_arm_dh()
        l = [dh[0]["d"], dh[1]["a"], dh[2]["a"], dh[3]["a"]]
        turtle_joint_offests = [0, (math.pi/2 - dh[1]["theta"]), - dh[2]["theta"], 0]

        ## 1st test
        ## q = np.array([0 pi/4 pi/4 pi/4]) # Test joint configuration
        request_position = np.matrix('-3.36652257913489e-01; 1.38399902569693e-18; 9.91024193532457e-02')
        py = {'pitch': -3*math.pi/4, 'yaw': 0}
        qs = inverse_kinematics_tb(request_position, py, l)  
        if len(qs) != 2 or (np.shape(qs[0]) != (len(dh),) or np.shape(qs[1]) != (len(dh),)):
            try:
                self.assertTrue(False)
            except AssertionError as e: 
                print("\n\t---\n\tTest [" + self.testName + "] failes:\n\tInverse kinematics result has wrong dimensions!\n\t---")
                self.testPassed = False

        ## Regard angle offsets for turtle bot
        qs[0] = normalize_angles(qs[0] + turtle_joint_offests)
        qs[1] = normalize_angles(qs[1] + turtle_joint_offests)
        
        TM = forward_kinematics_dh(dh, qs[0])
        position_error_one = linalg.norm(request_position - get_tcp_position(TM))
        TM = forward_kinematics_dh(dh, qs[1])
        position_error_two = linalg.norm(request_position - get_tcp_position(TM))
        print("\n\tInfo: Test [" + self.testName + "]:\n\tposition error of both elbow configurations:\n\t\t" + str(position_error_one) + "\n\t\t" + str(position_error_two))

        try: 
            self.assertAlmostEqual(position_error_one + position_error_two, 0.0, 6)
            print("\n\t---\n\tTest [" + self.testName + "] ok:\n\tTest position reached!" + "\n\t---")
        except AssertionError as e: 
            print("\n\t---\n\tTest [" + self.testName + "] failes:\n\tTest position not reached!" + "\n\t---")
            self.testPassed = False     
        
if __name__ == '__main__':
    import rostest
    print("n\t===\n\t==> Testing solution of:" + str(mat_nr()) + "\n\t===\n")
    rostest.rosrun(PKG, 'test_public', TestPublic, '--text')