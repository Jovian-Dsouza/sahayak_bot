#!/usr/bin/env python

'''
The goal of this script is to calculate the max acceleration and velocity 
of the robot

ax_max - max acc in x direction
vx_max - max vel in x direction

atheta_max - max angular acc
vtheta_max - max angular speed


'''

import rospy
import math
from math import pi

from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from gazebo_msgs.msg import ModelStates

class Robot:
    def __init__(self):
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_cb)
        #Current robot values
        self.x = 0.0
        self.theta = 0.0
        self.vx = 0.0
        self.vt = 0.0
        self.ax = 0.0
        self.at = 0.0

        self.vx_prev = 0.0
        self.vt_prev = 0.0

        self.ax_max = 0.0
        self.vx_max = 0.0
        self.at_max = 0.0
        self.vt_max = 0.0

    def model_states_cb(self, msg):
        # print(msg.name[-1]) #The last model is ebot
        pose = msg.pose[-1]
        twist = msg.twist[-1]
        # rospy.loginfo(pose)
        # rospy.loginfo(twist)
        # self.x = pose.position.x
        # self.theta = pose.orientation.z
        self.vx = math.sqrt(twist.linear.x **2 + twist.linear.y **2)
        self.vt = twist.angular.z

    def compute_acc(self):
        self.ax = (self.vx - self.vx_prev) * freq
        self.at = (self.vt - self.vt_prev) * freq

        self.vx_prev = self.vx
        self.vt_prev = self.vt
    
    def compute_max(self):
        self.vx_max = max(self.vx_max, self.vx)
        self.vt_max = max(self.vt_max, self.vt)
        self.at_max = max(self.at_max, self.at)
        self.ax_max = max(self.ax_max, self.ax)
    
    def print_state(self):
        print("vx = %0.4f, vt = %0.4f, ax = %0.4f, at = %0.4f" % \
            (self.vx, self.vt, self.ax, self.at))

    def print_max(self):
        rospy.loginfo("MAX")
        rospy.loginfo("max_vel_x: %0.5f" % self.vx_max)
        rospy.loginfo("max_vel_theta: %0.5f" % self.vt_max)
        rospy.loginfo("acc_lim_x: %0.5f" % self.ax_max)
        rospy.loginfo("acc_lim_theta: %0.5f" % self.at_max)



if __name__ == '__main__':
    rospy.init_node('calibration')
    robot = Robot()
    freq = 100
    rate = rospy.Rate(freq) #Loop running at 10 hz

    while not rospy.is_shutdown():
        try:
            robot.compute_acc()
            robot.compute_max()
            robot.print_state()
            rate.sleep()
        except rospy.exceptions.ROSInterruptException:
            pass
    robot.print_max()

    