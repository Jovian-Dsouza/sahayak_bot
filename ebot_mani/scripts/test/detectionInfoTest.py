#!/usr/bin/env python

import rospy
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty, EmptyRequest

def callback(msg):
    detection_obj[msg.name] = msg.pose.pose.position
    rospy.loginfo(detection_obj)

if __name__ == "__main__":
    rospy.init_node("detection_test")
    rospy.wait_for_service('/detect')
    detect = rospy.ServiceProxy('detect', Empty)
    detection_obj = {}
    rospy.Subscriber("/detection_info", ObjectPose, callback)
    detect()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        print(detection_obj.keys())
        rate.sleep() 