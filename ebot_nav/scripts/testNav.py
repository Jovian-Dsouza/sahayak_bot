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
from poses import poses

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
        self.client.send_goal_and_wait(self.goal, rospy.Duration(60), rospy.Duration(60))
        #self.client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)

    def done_cb(self, status, result):
        if status == 2: #Cancel request
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
            return 

        if status == 3: #Goal Reached
            rospy.loginfo("Goal pose reached") 
            return

        if status == 4: #Goal was aborted
            rospy.loginfo("Goal pose was aborted by the Action Server")
            return

        if status == 5: #Rejected by action server
            rospy.loginfo("Goal pose has been rejected by the Action Server")
            return

        if status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass

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

def go_to_goal(goal_name):
    start_time = rospy.Time.now()
    ebot.go_to_pose(poses[goal_name])
    stop_time = rospy.Time.now()
    rospy.loginfo("Time Taken %0.4f" % (stop_time-start_time).to_sec())

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

    # goalPose = create_2d_pose(6.01377, -0.5552, -pi/2)
    # go_to_goal(goalPose)

    # startPose = create_2d_pose(0, 0, 0)
    # go_to_goal(startPose)

    

    # go_to_goal('store_table')
    # go_to_goal('conference_dropbox')
    # go_to_goal('start')
    go_to_goal('pantry_entry')
    go_to_goal('pantry_table1')
    go_to_goal('pantry_table2')
    go_to_goal('pantry_exit')
    go_to_goal('start')
