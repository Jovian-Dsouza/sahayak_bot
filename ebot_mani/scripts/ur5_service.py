#! /usr/bin/env python

import rospy
from ebot_mani.ur5_helper import Ur5Moveit
from ebot_mani.srv import *

def set_named_pose_cb(req):
    flag = ur5.go_to_named_pose(req.poseName)
    return SetNamedPoseResponse(flag)

def set_pose_cb(req):
    flag = ur5.go_to_pose(req.pose)
    return SetPoseResponse(flag)

if __name__ == '__main__':
    rospy.init_node("ur5_service")
    ur5 = Ur5Moveit()

    rospy.Service('ebot_mani/set_named_pose', SetNamedPose, set_named_pose_cb)
    rospy.Service('ebot_mani/set_pose', SetPose, set_pose_cb)

    rospy.spin()

    del ur5

