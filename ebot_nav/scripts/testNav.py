#!/usr/bin/env python

import rospy
import math
from math import pi

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

# from ebot_mani.ur5_helper import Ur5Moveit

class Ebot():

    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        rospy.loginfo("Connected to move base server")
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
    
    def go_to_pose(self, goalPose):
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        self.goal.target_pose.pose = goalPose
        rospy.loginfo("Sending goal pose to Action Server")
        self.client.send_goal_and_wait(self.goal, rospy.Duration(30), rospy.Duration(30))


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

if __name__ == '__main__':
    # try:
    #     MoveBaseSeq()
    # except rospy.ROSInterruptException:
    #     rospy.loginfo("Navigation finished.")
    rospy.init_node('NavTest')
    ebot = Ebot()
    # ur5 = Ur5Moveit()
    # ur5.navigationPose()
    # del ur5
    start_time = rospy.Time.now()
    goalPose = create_2d_pose(6.01377, -0.5552, -pi/2)
    ebot.go_to_pose(goalPose)
    stop_time = rospy.Time.now()
    rospy.loginfo("Time Taken %0.4f" % (stop_time-start_time).to_sec())

    start_time = rospy.Time.now()
    goalPose = create_2d_pose(0, 0, 0)
    ebot.go_to_pose(goalPose)
    stop_time = rospy.Time.now()
    rospy.loginfo("Time Taken %0.4f" % (stop_time-start_time).to_sec())