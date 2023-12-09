import sys
import numpy as np
from copy import deepcopy
from math import pi

import rospy
# Common interfaces for interacting with both the simulation and real environments!
from core.interfaces import ArmController
from core.interfaces import ObjectDetector

# for timing that is consistent with simulation or real time as appropriate
from core.utils import time_in_seconds

from lib.IK_position_null import IK
from lib.calculateFK import FK

from StaticGrabber import StaticGrabber
from DynamicGrabber import DynamicGrabber


if __name__ == "__main__":
    try:
        team = rospy.get_param("team") # 'red' or 'blue'
    except KeyError:
        print('Team must be red or blue - make sure you are running final.launch!')
        exit()

    rospy.init_node("team_script")
    arm = ArmController()
    arm.set_arm_speed(0.3)
    arm.set_gripper_speed(0.2)
    arm.open_gripper()
    gripper_state = arm.get_gripper_state()
    print("gripper_state:",gripper_state['position'])
    detector = ObjectDetector()
    ik = IK()
    fk = FK()
    static_grabber = StaticGrabber(detector, arm, team, ik, fk)
    dynamic_grabber = DynamicGrabber(detector, arm, team, ik, fk)

    start_position = np.array([-0.01779206, -0.76012354,  0.01978261, -2.34205014, 0.02984053, 1.54119353+pi/2, 0.75344866])
    arm.safe_move_to_position(start_position) # on your mark!

    print("\n****************")
    if team == 'blue':
        print("** BLUE TEAM  **")
    else:
        print("**  RED TEAM  **")
    print("****************")
    input("\nWaiting for start... Press ENTER to begin!\n") # get set!
    print("Go!\n") # go!
    
    print("grabbing static blocks!!\n")
    for i in range(4):
        
        # 1. move to an over block position 
        static_grabber.move_to_over(i)
        # 2. detect the static blocks
        result_list = static_grabber.detect_and_convert()
        if len(result_list) == 0:
            print("redetecting!")
            i -= 1
            continue
        target_H = static_grabber.find_closest(result_list)
        # 3. grab the closest one
        static_grabber.grab(target_H, i)
        # 4. put to appropriate position
        static_grabber.put(i)
        print("successfully grab " + str(i+1)+ " block!!\n")
        
    arm.open_gripper()
        
    print("grabbing dynamic blocks!!\n")
    for i in range(4):
        
        # 1. move to pre pose
        dynamic_grabber.move_to_pre_pose()
        # 2. move to wait pose
        dynamic_grabber.move_to_wait_pose()
        # 3. wait until grabbed
        dynamic_grabber.wait_unitl_grabbed()
        # 4. move to initial pose
        dynamic_grabber.move_to_initial_pose()
        # 5. put at the static platform
        dynamic_grabber.put()
        for j in range(1):
            dynamic_grabber.get_over()
            # 6. detect the static blocks
            result_list = static_grabber.detect_and_convert()
            if len(result_list) == 0:
                print("redetecting!")
                j -= 1
                continue
            target_H = static_grabber.find_closest(result_list)
            # 7. grab the closest one
            static_grabber.grab(target_H, i)
            # 8. put to appropriate position
            static_grabber.put(i+4)
            dynamic_grabber.go_to_side()
            print("successfully grab " + str(i+5)+ " block!!\n")

