#! /usr/bin/env python

import rospy
from ebot_mani.ur5_helper import Ur5Moveit
from ebot_mani.srv import *
from std_srvs.srv import Empty, EmptyResponse

def set_named_pose_cb(req):
    flag = ur5.go_to_named_pose(req.poseName)
    return SetNamedPoseResponse(flag)

def print_name_pose_cb(req):
    flag = ur5.print_name_pose(req.poseName)
    return SetNamedPoseResponse(flag)

def set_pose_cb(req):
    flag = ur5.go_to_pose(req.pose)
    return SetPoseResponse(flag)

def set_pose_wrist_cb(req):
    flag = ur5.go_to_pose_wrist(req.pose)
    return SetPoseResponse(flag)

def set_pose_wrist_no_align_cb(req):
    flag = ur5.go_to_pose_wrist_no_align(req.pose)
    return SetPoseResponse(flag)


def set_pose_odom_cb(req):
    flag = ur5.go_to_pose_odom(req.pose)
    return SetPoseResponse(flag)

def set_pose_relative_cb(req):
    flag = ur5.go_to_pose_relative(req.pose)
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

def add_plane_cb(req):
    # ur5.scene.add_plane(req.name, req.pose, normal=(0, 0, 1), offset=0)
    ur5.scene.add_box(req.name, req.pose, size=(0.1, 0.1, 0.1))
    print("Add Plane" + req.name)
    return AddPlaneResponse()

def get_current_pose_odom_cb(req):
    resp = GetPoseResponse()
    resp.pose = ur5.getCurrentPoseOdom()
    return resp

def align_wrist_cb(req):
    ur5.align_wrist()
    return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node("ur5_service")
    ur5 = Ur5Moveit()

    rospy.Service('ebot_mani/set_named_pose', SetNamedPose, set_named_pose_cb)
    rospy.Service('ebot_mani/set_pose', SetPose, set_pose_cb)
    rospy.Service('ebot_mani/set_pose_wrist', SetPose, set_pose_wrist_cb)
    rospy.Service('ebot_mani/set_pose_wrist_no_align', SetPose, set_pose_wrist_no_align_cb)
    rospy.Service('ebot_mani/set_pose_odom', SetPose, set_pose_odom_cb)
    rospy.Service('ebot_mani/set_pose_relative', SetPose, set_pose_relative_cb)
    rospy.Service('ebot_mani/set_gripper', SetGripper, set_gripper_cb)
    rospy.Service('ebot_mani/open_gripper', Empty, open_gripper_cb)
    rospy.Service('ebot_mani/grasp_object_vertical', GraspObject, grasp_object_vertical_cb)
    rospy.Service('ebot_mani/grasp_object_horizontal', GraspObject, grasp_object_horizontal_cb)
    rospy.Service('ebot_mani/print_name_pose', SetNamedPose, print_name_pose_cb)
    rospy.Service('ebot_mani/get_current_pose_odom', GetPose ,get_current_pose_odom_cb)
    rospy.Service('ebot_mani/align_wrist', Empty, align_wrist_cb)

    rospy.Service('ebot_mani/add_plane', AddPlane, add_plane_cb)
    # rospy.Service('ebot_mani/add_box', AddBox, add_box_cb)
    rospy.spin()

    del ur5

