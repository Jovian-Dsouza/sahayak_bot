#! /usr/bin/env python

import rospy
import random
import math
from math import pi

from geometry_msgs.msg import Point, Quaternion, Pose , PointStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from ebot_mani.ur5_helper import Ur5Moveit

import resetModels

def roll_angle_to_quaternion(roll_angle):
    '''Returns a quaternion from roll angle(in deg)'''
    angle = (math.pi/180) * roll_angle
    q = quaternion_from_euler(-math.pi+angle, 0 , -math.pi)
    o = Quaternion()
    o.x , o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o

def angle_to_quaternion(roll, yaw):
    '''Returns a quaternion from roll angle(in deg)'''
    roll_angle = (math.pi/180) * roll
    yaw_angle = (math.pi/180) * yaw
    q = quaternion_from_euler(-math.pi+roll_angle, 0 , -math.pi+yaw_angle)
    o = Quaternion()
    o.x , o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o

class Dimension():
    def __init__(self, width, hight):
        self.width = width
        self.hight = hight

class Detect():
    def __init__(self):
        self.dict = {}
        rospy.wait_for_service('/detect')
        self.detect = rospy.ServiceProxy('detect', Empty)
        rospy.Subscriber("/detection_info", ObjectPose, self.detect_callback)

        self.dim_dict = {}
    
    def detect_callback(self, msg):
        self.dict[msg.name] = msg.pose.pose.position
        self.frame_id = msg.pose.header.frame_id
        rospy.loginfo(">> Detected : " + msg.name)

        #TODO Remove before submission
        self.dim_dict[msg.name] = Dimension(msg.pose.pose.orientation.w, msg.pose.pose.orientation.z)
        # rospy.loginfo("Estimated grasping width : " + str( self.width_dict[msg.name] + 0.2))


if __name__ == '__main__':

    rospy.init_node('ObjectAngleDetection')
    
    model = 'eYFi_board'
    #model = 'FPGA_board'

    resetModels.wait_for_all_services()
    resetModels.delete_all_model()
    spawn_angle = 0.0 #In radians

    if model == 'eYFi_board':
        resetModels.spawn_eYIFI(y=3.2, angle=spawn_angle)
    if model == 'FPGA_board':
        resetModels.spawn_FPGA(y=3.2, angle=spawn_angle)
    
    rospy.sleep(4)

    detect = Detect()
    detect.detect() #Call the detect service

    print(model)
    print("Width : " + str(detect.dim_dict[model].width))
    print("hight : " + str(detect.dim_dict[model].hight))
    #print(detect.dict[model]) #centroid
    distance_from_object = detect.dict[model].y
    print("Distance : " + str(distance_from_object))

    #Calculate and print the angle

    #print(angle)