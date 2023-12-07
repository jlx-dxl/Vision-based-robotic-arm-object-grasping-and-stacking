import sys
import numpy as np
from copy import deepcopy
from math import pi
from time import sleep

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

from lib.IK_position_null import IK
from lib.calculateFK import FK


class DynamicGrabber():
    def __init__(self, detector, arm, team, ik, fk):
        self.detector = detector
        self.arm = arm
        self.team = team
        self.ik = ik
        self.fk = fk
        if team == 'blue':
        # 1 7 joint, minus is close to blue des, plus is to static blocks
        # 2 joint, plus is bend down, minus is lay up
        # 4 6, plus is look up, minus is look down
            # self.over_blk = np.array([-0.01779206+0.4, -0.76012354+0.65,  0.01978261, -2.34205014+0.4, 0.02984053, 1.54119353+0.35, 0.75344866+0.4])
            self.pre_pose = np.array([ 0.3271, -1.2986, -1.8317, -1.4963, -0.1884,  1.8406, -0.8989])
            self.wait_pose = np.array([ 0.6789, -1.4364, -1.7766, -1.1599, -0.3011,  1.8446, -0.7817])
            self.setpoint = np.array([
                                [-0.0975,  0.2073, -0.1692, -2.0558,  0.0449,  2.2597,  0.4937] ,
                                [-0.1087,  0.1437, -0.1578, -2.0025,  0.0268,  2.1443,  0.506 ] ,
                                [-0.1168,  0.0973, -0.1489, -1.9295,  0.016 ,  2.0256,  0.5133] ,
                                [-0.1218,  0.0688, -0.1432, -1.8359,  0.0104,  1.904 ,  0.5173] ,
                                [-0.1241,  0.0593, -0.1411, -1.7201,  0.0085,  1.7789,  0.5186] ,
                                [-0.1248,  0.0708, -0.1425, -1.578 ,  0.0101,  1.6482,  0.5177] ,
                                [-0.1261,  0.1076, -0.1469, -1.401 ,  0.0158,  1.5075,  0.5143] ,
                                [-0.1344,  0.1814, -0.1515, -1.1668,  0.0279,  1.3463,  0.5082] ,
                                ])
            self.reputpoint = np.array([ 0.2531,  0.1982,  0.0518, -2.0225, -0.0128,  2.2204,  1.0971])
            self.redetectpoint = np.array([ 0.1888,  0.0769,  0.1193, -1.6597, -0.0093,  1.736 ,  1.0947])
        else:
            # 1 7 joint, plus is close to red des, minus is to static blocks
            # 2 joint, plus is bend down, minus is lay up
            # 4 6, plus is look up, minus is look down
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65,  0.01978261, -2.34205014+0.4, 0.02984053, 1.54119353+0.35, 0.75344866-0.4])
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65+0.1,  0.01978261, -2.34205014+0.4+0.2, 0.02984053, 1.54119353+0.35-0.15, 0.75344866-0.4])
            self.pre_pose = np.array([ 0.6241,  0.861 ,  0.748 , -1.482 ,  0.4491,  1.7816, -1.2659])
            self.wait_pose = np.array([ 0.9194,  1.0436,  0.7226, -1.1527,  0.4315,  1.8183, -1.1073])
            self.setpoint = np.array([
                                [ 0.2318,  0.2045,  0.0301, -2.056 , -0.0079,  2.2604,  1.0517] ,
                                [ 0.1986,  0.1423,  0.0645, -2.0026, -0.0109,  2.1445,  1.0538] ,
                                [ 0.1755,  0.0966,  0.0882, -1.9295, -0.0095,  2.0257,  1.0528] ,
                                [ 0.1619,  0.0684,  0.102 , -1.8359, -0.0074,  1.904 ,  1.0514] ,
                                [ 0.1574,  0.0591,  0.1067, -1.7201, -0.0064,  1.7789,  1.0507] ,
                                [ 0.1628,  0.0705,  0.1026, -1.5781, -0.0072,  1.6482,  1.0512] ,
                                [ 0.1799,  0.107 ,  0.0881, -1.4011, -0.0094,  1.5076,  1.0524] ,
                                [ 0.2123,  0.1799,  0.058 , -1.1669, -0.0106,  1.3465,  1.0525] ,
                                ])
            self.reputpoint = np.array([-0.1557,  0.1977, -0.2009, -2.028 ,  0.0493,  2.2213,  0.4028])
            self.redetectpoint = np.array([-0.1467,  0.0774, -0.1634, -1.6596,  0.0127,  1.736 ,  0.4737])
            
        self.H_ee_camera = detector.get_H_ee_camera()
        
        
    def move_to_pre_pose(self):
        self.arm.safe_move_to_position(self.pre_pose)
          
            
    def move_to_wait_pose(self):
        self.arm.safe_move_to_position(self.wait_pose)
        
        
    def wait_unitl_grabbed(self):
        for i in range(5):
            sleep(6)
            state = self.arm.exec_gripper_cmd(0.04, 80)
            if state == True:
                print("nothing grabbed for " + str(i) + " times\n")
                self.arm.open_gripper()
                continue
            else:
                print("successfully grabbed one block")
                break
            
            
    def move_to_initial_pose(self):
        self.arm.safe_move_to_position(np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866]))
    
    
    def put(self):
        self.arm.safe_move_to_position(self.reputpoint)
        self.arm.open_gripper()
        self.arm.safe_move_to_position(self.redetectpoint)
        
    
        
    
        
    
        
        
