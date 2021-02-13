#! /usr/bin/env python

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import pi

def create_pose_quaternion(x, y, z ,qx ,qy ,qz ,qw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def create_pose_euler(x, y, z, roll, pitch, yaw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = quaternion_from_euler(roll, pitch , yaw)

    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

joint_angles_dict = {
    "homePose" : [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ],
    "homePose1" : [0.0, -0.5901, -0.2, -1.3314, -0.5235, 1.5708 ],
    "navPose" : [0.0, -0.7290, 0.2256, 0.5207, 1.5700, 0.0],
    "frontHomePose" : [0.0, -0.3124, -0.83, 1.0067, 1.5700, 0.0 ]
}

poses_dict = { #w.r.t base_link -0.65
    "homePoseNew" : create_pose_euler(-0.65 , -0.0, 0.6, -pi + 15 *(pi/180), 0.0, -pi-pi/2),
}