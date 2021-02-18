#! /usr/bin/env python

'''
This node uses the detection_info topic and performs the actual Ur5 arm manipulation
'''

import rospy
import random
from math import pi

from geometry_msgs.msg import Point, Quaternion, Pose , PointStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from ebot_mani.srv import *
from testNav import Ebot


def roll_angle_to_quaternion(roll_angle):
    '''Returns a quaternion from roll angle(in deg)'''
    angle = (math.pi/180) * roll_angle
    q = quaternion_from_euler(-math.pi+angle, 0 , -math.pi)
    o = Quaternion()
    o.x , o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o

def angle_to_quaternion(roll, yaw):
    '''Returns a quaternion from roll angle(in deg) and yaw angle'''
    roll_angle = (math.pi/180) * roll
    yaw_angle = (math.pi/180) * yaw
    q = quaternion_from_euler(-math.pi+roll_angle, 0 , -math.pi+yaw_angle)
    o = Quaternion()
    o.x , o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o

class Detect():
    def __init__(self):
        self.dict = {}
        rospy.wait_for_service('/detect')
        self.detect = rospy.ServiceProxy('detect', Empty)
        rospy.Subscriber("/detection_info", ObjectPose, self.detect_callback)
    
    def detect_callback(self, msg):
        self.dict[msg.name] = msg.pose.pose.position
        self.frame_id = msg.pose.header.frame_id
        rospy.loginfo(">> Detected : " + msg.name)

        #TODO Remove before submission
        rospy.loginfo("Estimated grasping width : " + str(msg.pose.pose.orientation.w + 0.2))

class Ur5():
    def __init__(self):
        rospy.wait_for_service('ebot_mani/set_named_pose')
        rospy.wait_for_service('ebot_mani/set_pose')
        rospy.wait_for_service('ebot_mani/set_gripper')
        rospy.wait_for_service('ebot_mani/open_gripper')
        rospy.wait_for_service('ebot_mani/grasp_object_vertical')
        rospy.wait_for_service('ebot_mani/grasp_object_horizontal')
        rospy.loginfo("connected to services")

        self.go_to_named_pose = rospy.ServiceProxy('ebot_mani/set_named_pose', SetNamedPose)
        self.go_to_pose = rospy.ServiceProxy('ebot_mani/set_pose', SetPose)
        self.closeGripper = rospy.ServiceProxy('ebot_mani/set_gripper',SetGripper)
        self.openGripper = rospy.ServiceProxy('ebot_mani/open_gripper',Empty)
        self.graspObjectVerticalService = rospy.ServiceProxy('ebot_mani/grasp_object_vertical', GraspObject)
        self.graspObjectHorizontalService = rospy.ServiceProxy('ebot_mani/grasp_object_horizontal', GraspObject)
        
    def graspObjectHorizontal(self, point, width, yaw=0):
        req = GraspObjectRequest()
        req.point = point
        req.width = width
        req.yaw = yaw
        return self.graspObjectHorizontalService(req)
    
    def graspObjectVertical(self, point, width, yaw):
        req = GraspObjectRequest()
        req.point = point
        req.width = width
        req.yaw = yaw
        return self.graspObjectVerticalService(req)
 
def pickupObject(object_name):
    '''
    Note :  object_name should be the real model name and not the gazebo model name
    '''
    ur5.openGripper()

    if object_name == 'eYFi_board':
        #TODO need a better way of finding the object's yaw angle instead of manually giving it
        return ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/4)
    elif object_name == 'FPGA_board':
        resp = ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/3)
        # resp = ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=0)
        return resp.success
        
    else:
        return ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=0)

    # ur5.go_to_named_pose("navPose")

if __name__ == '__main__':

    #width estimate  = 0.2 + width of detection window (printed in terminal) 
    #w_dict uses real model names
    w_dict = {'coke_can': 0.27086, 
              'battery': 0.26500, 
              'glue' : 0.31,
              'eYFi_board' : 0.5,
              'adhesive' : 0.267674286664,
              'water_glass' : 0.2,
              'robot_wheels' : 0.26,
              'FPGA_board' : 0.3
              } 

    rospy.init_node('grasping_node')

    ur5 = Ur5()
    ebot = Ebot()

    ur5.go_to_named_pose("storeHomePose")
    
    detect = Detect()
    detect.detect() #Call the detect service
    
    ebot.go_to_goal("store_table_front")
    y = detect.dict['FPGA_board'].y
    x = 1

    ebot.go_to_pose_relative(x, y, 0)
    print("(%0.4f, %0.4f)" % (-x, -y))
    
    
    ur5.go_to_named_pose("seeObjectPose")
    detect.detect()
    rospy.sleep(2)

    print(pickupObject('FPGA_board'))
    # while pickupObject('FPGA_board') == False:
    #     detect.dict['FPGA_board'].z += 0.05
    #     rospy.sleep(2)
   

    # pickupObject('battery')

    

