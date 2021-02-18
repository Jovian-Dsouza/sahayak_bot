#!/usr/bin/env python

import rospy
from testNav import Ebot
import sys

if __name__ == '__main__':
    rospy.init_node('namePose')
    ebot = Ebot()

    if len(sys.argv) == 1:
        print("Applying Brakes")
        ebot.applyBrakes()

    if len(sys.argv) == 2:
        print("Going to goal")
        ebot.go_to_goal(sys.argv[1])
        ebot.print_current_pose()
    