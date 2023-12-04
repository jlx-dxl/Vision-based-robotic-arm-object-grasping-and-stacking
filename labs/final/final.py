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
# from DynamicGrabber import DynamicGrabber


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
    detector = ObjectDetector()
    ik = IK()
    fk = FK()
    static_grabber = StaticGrabber(detector, arm, team, ik, fk)

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

    # STUDENT CODE HERE
    # static_grabber.move_to_over()
    
    # # 2. detect the static blocks
    # result_list_origin = static_grabber.detect_and_convert()
    
    for i in range(4):
        
        static_grabber.move_to_over(i)
        # 2. detect the static blocks
        result_list = static_grabber.detect_and_convert()
        
        target_H = static_grabber.find_closest(result_list)
        # 3. grab the closest one
        static_grabber.grab(target_H, i)
        # 4. put to appropriate position
        static_grabber.put(i)
        # # 5. move back to the start point
        # static_grabber.move_to_over(i)
        
    # Move around...

    # END STUDENT CODE