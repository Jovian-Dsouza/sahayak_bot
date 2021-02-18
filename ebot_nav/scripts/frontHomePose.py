#!/usr/bin/env python

import rospy
from ebot_mani.ur5_helper import Ur5Moveit

class Ur5(Ur5Moveit):
    def __init__(self):
        Ur5Moveit.__init__(self)

    def homePose(self):
        home_joint_angles = [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ]
        self.set_joint_angles(home_joint_angles)
        rospy.loginfo('\033[94m' + " >>> Ur5 is homed " + '\033[0m')
    
    def homePose1(self):
        home_joint_angles = [0.0, -0.5901, -0.2, -1.3314, -0.5235, 1.5708 ]
        self.set_joint_angles(home_joint_angles)
        rospy.loginfo('\033[94m' + " >>> Ur5 is homed " + '\033[0m')

    def navPose(self):
        nav_joint_angles = [0.0, -0.7290, 0.2256, 0.5207, 1.5700, 0.0]
        self.set_joint_angles(nav_joint_angles)
        rospy.loginfo('\033[94m' + " >>> Ur5 is in NavPose " + '\033[0m')
    
    def frontHomePose(self):
        home_joint_angles = [0.0, -0.3124, -0.83, 1.0067, 1.5700, 0.0 ]
        self.set_joint_angles(home_joint_angles)
        rospy.loginfo('\033[94m' + " >>> Ur5 is frontHomePose " + '\033[0m')

if __name__ == '__main__':
    rospy.init_node("NavPose")
    ur5 = Ur5()
    #ur5.navPose()
    #ur5.frontHomePose()
    ur5.homePose1()
    del ur5
