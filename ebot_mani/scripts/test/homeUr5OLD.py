#! /usr/bin/env python

# This simple node just homes the ur5 arm for the perception nodes 
# Used in the launch file before calling the perception module

import rospy
from ur5_helper import Ur5Moveit

class Ur5(Ur5Moveit):
    def __init__(self):
        Ur5Moveit.__init__(self)

    def homePose(self):
        home_joint_angles = [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ]
        self.set_joint_angles(home_joint_angles)
        rospy.loginfo('\033[94m' + " >>> Ur5 is homed " + '\033[0m')

if __name__ == '__main__':
    rospy.init_node("homeUr5")
    ur5 = Ur5()
    ur5.homePose()
    del ur5

