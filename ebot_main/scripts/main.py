#! /usr/bin/env python

'''
This node uses the detection_info topic and performs the actual Ur5 arm manipulation
'''

import rospy
import random
from math import pi

from geometry_msgs.msg import Point, Quaternion, Pose , PointStamped, PoseStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from ebot_mani.srv import *
from testNav import Ebot

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

def createPoseStamped(point):
    poseStamped = PoseStamped()
    poseStamped.header.frame_id = 'base_link'
    poseStamped.header.stamp = rospy.Time.now()
    poseStamped.pose.position = point
    poseStamped.pose.orientation.x = 0 
    poseStamped.pose.orientation.y = -0.7071
    poseStamped.pose.orientation.z = 0
    poseStamped.pose.orientation.w = 0.7071
    return poseStamped

class Detect():
    def __init__(self):
        self.dict = {}
        rospy.loginfo("waiting for detect service")
        rospy.wait_for_service('/ebot/detect')
        self.detect_service = rospy.ServiceProxy('/ebot/detect', Empty)
        rospy.Subscriber("/detection_info", ObjectPose, self.detect_callback)
    
    def print_detected(self):
        for item in self.dict.keys():
            print(">> Detected : " + item)
    
    def detect(self):
        self.dict = {}
        self.detect_service()
        rospy.sleep(2)
        self.print_detected()

    def detect_callback(self, msg):
        self.dict[msg.name] = msg.pose.pose.position
        self.frame_id = msg.pose.header.frame_id
        # rospy.loginfo(">> Detected : " + msg.name)

        #TODO Remove before submission
        # rospy.loginfo("Estimated grasping width : " + str(msg.pose.pose.orientation.w + 0.2))

class Ur5():
    def __init__(self):
        rospy.loginfo("waiting for ur5_service")
        rospy.wait_for_service('ebot_mani/set_named_pose')
        rospy.wait_for_service('ebot_mani/set_pose')
        rospy.wait_for_service('ebot_mani/set_gripper')
        rospy.wait_for_service('ebot_mani/open_gripper')
        rospy.wait_for_service('ebot_mani/grasp_object_vertical')
        rospy.wait_for_service('ebot_mani/grasp_object_horizontal')
        rospy.wait_for_service('ebot_mani/set_pose_relative')
        rospy.loginfo("connected to services")

        self.go_to_named_pose = rospy.ServiceProxy('ebot_mani/set_named_pose', SetNamedPose)
        self.go_to_pose = rospy.ServiceProxy('ebot_mani/set_pose', SetPose)
        self.closeGripper = rospy.ServiceProxy('ebot_mani/set_gripper',SetGripper)
        self.openGripper = rospy.ServiceProxy('ebot_mani/open_gripper',Empty)
        self.graspObjectVerticalService = rospy.ServiceProxy('ebot_mani/grasp_object_vertical', GraspObject)
        self.graspObjectHorizontalService = rospy.ServiceProxy('ebot_mani/grasp_object_horizontal', GraspObject)
        self.go_to_pose_relative = rospy.ServiceProxy('ebot_mani/set_pose_relative', SetPose)
        self.getCurrentPoseOdom = rospy.ServiceProxy('ebot_mani/get_current_pose_odom', GetPose)
        self.go_to_pose_odom = rospy.ServiceProxy('ebot_mani/set_pose_odom', SetPose)
        
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
    graspPose_pub.publish(createPoseStamped(detect.dict[object_name]))

    if object_name == 'eYFi_board':
        #TODO need a better way of finding the object's yaw angle instead of manually giving it
        return ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/4).success
    elif object_name == 'FPGA_board':
        return ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/3).success
        # resp = ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=0)
    else:
        return ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=0).success


def main():
    ur5.go_to_named_pose("navPose")
    ebot.go_to_goal('store_table')
    print("Reached Goal")
    ebot.print_current_pose()

    ur5.go_to_named_pose("storeHomePose")
    while ur5.go_to_named_pose("storeHomePoseOdom").success == False:
        rospy.logerr("going to pose failed")
        rospy.sleep(0.5)

    # currPose = ur5.getCurrentPoseOdom().pose
    # print(currPose)
    
    detect.detect()

    x = 0.8
    y = detect.dict['FPGA_board'].y
    ebot.go_to_pose_relative(x, y, 0)

    # while ur5.go_to_named_pose("seeObjectPose").success == False:
    #     rospy.logerr("going to pose failed")
    #     rospy.sleep(0.5)

    while ur5.go_to_named_pose("storeHomePose").success == False:
        rospy.logerr("going to pose failed")
        rospy.sleep(0.5)

    detect.detect()

if __name__ == '__main__':

    rospy.init_node('grasping_node')

    graspPose_pub = rospy.Publisher("/graspPose", PoseStamped, queue_size=1)

    ur5 = Ur5()
    ebot = Ebot()
    detect = Detect()

    # main()
    ur5.go_to_named_pose("storeHomePose")
    # ur5.go_to_named_pose("frontHomePoseNew")
    detect.detect()
    
    # ur5.go_to_named_pose("homePose")
    
    # pickupObject('battery')
    pickupObject('FPGA_board')
        
    