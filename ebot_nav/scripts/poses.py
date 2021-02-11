#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

def create_2d_pose(x, y , theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0

    q = quaternion_from_euler(0,0, theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

poses = {
        'start' : create_2d_pose(0.0, 0.0, 0.0),
        'store_table' : create_2d_pose(25.70008, -3.05864 ,-0.62764),
        'conference_dropbox' : create_2d_pose(6.03440, -0.58368 ,-1.58053),
        'pantry_entry' : create_2d_pose(13.14010, 1.19538 ,-1.58106),
        'pantry_table1' : create_2d_pose(14.44082, -0.80389 ,-0.01985),
        'pantry_table2' : create_2d_pose(11.14661, -1.10000 ,-3.11092),
        'pantry_exit' : create_2d_pose(13.03725, -0.30450 ,1.56538)
    }