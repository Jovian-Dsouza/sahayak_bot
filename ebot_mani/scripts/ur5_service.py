#! /usr/bin/env python

import rospy
from ebot_mani.ur5_helper import Ur5Moveit
from ebot_mani.srv import *
from std_srvs.srv import Empty, EmptyResponse

def set_named_pose_cb(req):
    flag = ur5.go_to_named_pose(req.poseName)
    return SetNamedPoseResponse(flag)

def set_pose_cb(req):
    flag = ur5.go_to_pose(req.pose)
    return SetPoseResponse(flag)

def set_gripper_cb(req):
    flag = ur5.set_gripper_value([req.width])
    return SetGripperResponse(flag)

def open_gripper_cb(req):
    ur5.openGripper()
    return EmptyResponse()

def grasp_object_vertical_cb(req):
    flag = ur5.graspObjectVertical(req.point, req.width, req.yaw)
    return GraspObjectResponse(flag)

def grasp_object_horizontal_cb(req):
    flag = ur5.graspObjectHorizontal(req.point, req.width, req.yaw)
    return GraspObjectResponse(flag)
    
if __name__ == '__main__':
    rospy.init_node("ur5_service")
    ur5 = Ur5Moveit()

    rospy.Service('ebot_mani/set_named_pose', SetNamedPose, set_named_pose_cb)
    rospy.Service('ebot_mani/set_pose', SetPose, set_pose_cb)
    rospy.Service('ebot_mani/set_gripper', SetGripper, set_gripper_cb)
    rospy.Service('ebot_mani/open_gripper', Empty, open_gripper_cb)
    rospy.Service('ebot_mani/grasp_object_vertical', GraspObject, grasp_object_vertical_cb)
    rospy.Service('ebot_mani/grasp_object_horizontal', GraspObject, grasp_object_horizontal_cb)
    rospy.spin()

    del ur5

