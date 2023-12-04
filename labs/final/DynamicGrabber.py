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
            self.pre_pose = 
            self.wait_pose = 
            self.setpoint = 
        else:
            # 1 7 joint, plus is close to red des, minus is to static blocks
            # 2 joint, plus is bend down, minus is lay up
            # 4 6, plus is look up, minus is look down
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65,  0.01978261, -2.34205014+0.4, 0.02984053, 1.54119353+0.35, 0.75344866-0.4])
            # self.over_blk = np.array([-0.01779206-0.4, -0.76012354+0.65+0.1,  0.01978261, -2.34205014+0.4+0.2, 0.02984053, 1.54119353+0.35-0.15, 0.75344866-0.4])
            self.pre_pose = 
            self.wait_pose = 
            self.setpoint = np.array([
                                [-0.3359,  0.5122,  0.2705, -2.0282,  1.4142,  1.6201, -1.729 ] ,
                                [-0.2938,  0.4019,  0.2214, -2.0502,  1.447 ,  1.5788, -1.6535] ,
                                [-0.2484,  0.3038,  0.1687, -2.0567,  1.4735,  1.5445, -1.5702] ,
                                [-0.2026,  0.219 ,  0.1161, -2.0476,  1.4943,  1.5183, -1.4807] ,
                                [-0.1594,  0.1481,  0.0675, -2.0227,  1.5102,  1.4999, -1.3868] ,
                                [-0.1212,  0.0913,  0.0256, -1.982 ,  1.5226,  1.4881, -1.2898] ,
                                [-0.09  ,  0.0489, -0.0075, -1.9253,  1.5327,  1.4809, -1.1906] ,
                                [-0.0682,  0.0213, -0.0302, -1.8524,  1.542 ,  1.4767, -1.0897]
                                ])
            
        self.H_ee_camera = detector.get_H_ee_camera()
        
        
    def move_to_pre_pose(self):
        self.arm.safe_move_to_position(self.pre_pose)
          
            
    def move_to_wait_pose(self):
        self.arm.safe_move_to_position(self.wait_pose)
        
        
    def wait(self):
        sleep(10)
        
        
    def grab(self):
        self.arm.exec_gripper_cmd(0.045, 80)
    
    
    def put(self,i):
        self.arm.safe_move_to_position(self.set_point[i,:])
        self.arm.open_gripper()
        # define a safe_position
        # safe_position = deepcopy(self.set_point[i,:])
        # safe_position[3] += 0.2
        # safe_position[5] -= 0.2
        # self.arm.safe_move_to_position(safe_position)
        
    
        
    
        
        
