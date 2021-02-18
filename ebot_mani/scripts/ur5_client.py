#! /usr/bin/env python

import rospy
from ebot_mani.srv import *
from ebot_mani.mani_poses import *
import sys

if __name__ == '__main__':
    
    rospy.wait_for_service('ebot_mani/set_named_pose')
    rospy.wait_for_service('ebot_mani/set_pose')
    rospy.wait_for_service('ebot_mani/print_name_pose')
    rospy.loginfo("connected to services")

    set_named_pose = rospy.ServiceProxy('ebot_mani/set_named_pose', SetNamedPose)
    set_pose = rospy.ServiceProxy('ebot_mani/set_pose', SetPose)
    print_name_pose = rospy.ServiceProxy('ebot_mani/print_name_pose', SetNamedPose)

    if len(sys.argv) < 2:
        print(">> Pre Definded Pose list")

        print("JOINT ANGLE")
        for name in joint_angles_dict.keys():
            print(name)
        print("")

        print("POSES")
        for name in poses_dict.keys():
            print(name)
        print("")

        print("ODOM POSES")
        for name in odom_poses_dict.keys():
            print(name)
        print("")

    else:
        print(print_name_pose(sys.argv[1]))


   
    

