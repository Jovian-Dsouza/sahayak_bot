#!/usr/bin/env python

from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from math import pi

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
        'test1' : create_2d_pose(13.35541, 3.11416 ,0.00236),
        'test2' : create_2d_pose(19.24265, -0.48021 ,-0.61752),

        'store_exit' : create_2d_pose(18.15108, 0.32930 ,2.71475),
        'store_table' : create_2d_pose(25.2, -3.0, -0.66687),
        'store_table_front' : create_2d_pose(25.58914, -3.26011, -0.64362),
        'store_table_close' : create_2d_pose(25.88223, -3.30938, -0.61585),
        'store_table_fpga' : create_2d_pose(25.90650, -3.23410, -0.60819),
        'conference_entry' : create_2d_pose(5.24608, 0.88915 ,-1.58584),
        # 'conference_entry' : create_2d_pose(5.22415, 0.09826 ,-1.55530),
        'conference_dropbox' : create_2d_pose(5.30199, -0.43187, -1.62417),
        'pantry_entry' : create_2d_pose(12.98767, 1.19538 ,-1.58106),
        'pantry_table2' : create_2d_pose(14.44082, -0.80389 ,-0.01985),
        'pantry_table1' : create_2d_pose(11.44625, -0.9500286, 3.13460),
        'pantry_exit_old' : create_2d_pose(13.03725, -0.30450 ,1.56538),
        'pantry_exit' : create_2d_pose(13.08767, -0.06591 ,1.54740),
        
        'meeting_entry' : create_2d_pose(8.68944, 1.12644 ,1.56883),
        'meeting_exit' : create_2d_pose(8.67912, 2.35998 ,-1.57724),
        'meeting_entry_old' : create_2d_pose(8.68885, 1.86032 ,1.60880),
        'meeting_table' : create_2d_pose(8.34436, 2.46636 ,1.57224),
        'meeting_dropbox':  create_2d_pose(7.24612, 2.50010, 1.58240),
        'research_entry' : create_2d_pose(10.68284, 6.66446 ,1.59342),
        'research_dropbox' : create_2d_pose(10.82558, 9.31412 ,-0.04216)
    }