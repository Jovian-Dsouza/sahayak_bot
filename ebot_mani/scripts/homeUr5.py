#! /usr/bin/env python

import rospy
from ebot_mani.srv import *

if __name__ == '__main__':
    
    rospy.wait_for_service('ebot_mani/set_named_pose')
    rospy.wait_for_service('ebot_mani/set_pose')
    rospy.loginfo("connected to services")

    set_named_pose = rospy.ServiceProxy('ebot_mani/set_named_pose', SetNamedPose)
    set_pose = rospy.ServiceProxy('ebot_mani/set_pose', SetPose)

    set_named_pose("homePose")
    rospy.loginfo("Home Pose set")
   
    

