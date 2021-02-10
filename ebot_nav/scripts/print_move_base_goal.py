#!/usr/bin/env python
from move_base_msgs.msg import MoveBaseActionGoal
from tf.transformations import euler_from_quaternion

import rospy

def goal_cb(msg):
    p = msg.goal.target_pose.pose.position
    q = msg.goal.target_pose.pose.orientation

    angles = euler_from_quaternion([q.x, q.y, q.z, q.w])

    print("create_2d_pose(%0.5f, %0.5f ,%0.5f)" % 
            (p.x, p.y, angles[2]))

if __name__ == '__main__':
    rospy.init_node('print_goal')
    print("(x, y, a)")
    rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, goal_cb)
    rospy.spin()