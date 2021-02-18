#! /usr/bin/env python

'''
This node uses the detection_info topic and performs the actual Ur5 arm manipulation
'''

import rospy
import random
from math import pi, sin, cos

from geometry_msgs.msg import Point, Quaternion, Pose, PointStamped, PoseStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from ebot_mani.srv import *
from testNav import Ebot

from perception.srv import *
transformPose = rospy.ServiceProxy('/get_transform_pose', GetTransformPose)
transformPoint = rospy.ServiceProxy('/get_transform_point', GetTransformPoint)

def TransformPoint(point, from_frame, to_frame):
    req = GetTransformPointRequest()
    req.point = point
    req.from_frame = from_frame
    req.to_frame = to_frame
    return transformPoint(req).point 

# width estimate  = 0.2 + width of detection window (printed in terminal)
# w_dict uses real model names
w_dict = {'coke_can': 0.27086,
          'battery': 0.26500,
          'glue': 0.31,
          'eYFi_board': 0.5,
          'adhesive': 0.267674286664,
          'water_glass': 0.2,
          'robot_wheels': 0.26,
          'FPGA_board': 0.3
          }


def printReached(name):
    print(">> " + name + " Reached")


def printPicked(name):
    print(">> " + name + " Picked")


def printDropped(name, dropbox):
    print(">> " + name + " Dropped in " + dropbox)


def printPoint(point):
    p = point
    print("create_point(%0.5f, %0.5f, %0.5f)" %
          (p.x, p.y, p.z))


def create_point(x, y, z):
    position = Point()
    position.x = x
    position.y = y
    position.z = z
    return position


def printPose(pose):
    p = pose.position
    q = pose.orientation
    print("create_pose_quaternion(%0.5f, %0.5f, %0.5f, %0.5f, %0.5f, %0.5f, %0.5f)" %
          (p.x, p.y, p.z, q.x, q.y, q.z, q.w))


def create_pose_quaternion(x, y, z, qx, qy, qz, qw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose


def orient_from_euler(roll, pitch, yaw):
    '''
    Input is roll, pitch, yaw
    output is Quaternion pose.orientation
    '''
    q = quaternion_from_euler(roll, pitch, yaw)
    o = Quaternion()
    o.x, o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o


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


def pickupObject(object_name):
    '''
    Note :  object_name should be the real model name and not the gazebo model name
    '''
    ur5.openGripper()
    graspPose_pub.publish(createPoseStamped(detect.dict[object_name]))

    if object_name == 'eYFi_board':
        # TODO need a better way of finding the object's yaw angle instead of manually giving it
        return ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/4).success
    elif object_name == 'FPGA_board':
        # return ur5.graspObjectVertical(detect.dict[object_name], width=w_dict[object_name], yaw=pi/3).success
        return ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=-pi/6)
    else:
        # .success
        return ur5.graspObjectHorizontal(detect.dict[object_name], width=w_dict[object_name], yaw=0)


class Detect():
    def __init__(self):
        self.dict = {}
        rospy.loginfo("waiting for detect service")
        rospy.wait_for_service('/ebot/detect')
        self.detectTable = rospy.ServiceProxy('/ebot/detectTable', Empty)
        self.detect_service = rospy.ServiceProxy('/ebot/detect', Empty)
        rospy.Subscriber("/detection_info", ObjectPose, self.detect_callback)

    def print_detected(self):
        for item in self.dict.keys():
            print(">> " + item + " Identified")

    def detect(self):
        self.dict = {}
        self.detect_service()
        rospy.sleep(2)
        self.print_detected()

    def detect_callback(self, msg):
        self.dict[msg.name] = msg.pose.pose.position
        self.frame_id = msg.pose.header.frame_id


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

        self.go_to_named_pose = rospy.ServiceProxy(
            'ebot_mani/set_named_pose', SetNamedPose)
        self.print_name_pose = rospy.ServiceProxy(
            '/ebot_mani/print_name_pose', SetNamedPose)
        self.go_to_pose = rospy.ServiceProxy('ebot_mani/set_pose', SetPose)
        self.closeGripper = rospy.ServiceProxy(
            'ebot_mani/set_gripper', SetGripper)
        self.openGripper = rospy.ServiceProxy('ebot_mani/open_gripper', Empty)
        self.graspObjectVerticalService = rospy.ServiceProxy(
            'ebot_mani/grasp_object_vertical', GraspObject)
        self.graspObjectHorizontalService = rospy.ServiceProxy(
            'ebot_mani/grasp_object_horizontal', GraspObject)
        self.set_pose_relative = rospy.ServiceProxy(
            'ebot_mani/set_pose_relative', SetPose)
        self.getCurrentPoseOdom = rospy.ServiceProxy(
            'ebot_mani/get_current_pose_odom', GetPose)
        self.set_pose_odom = rospy.ServiceProxy(
            'ebot_mani/set_pose_odom', SetPose)
        self.set_pose_wrist = rospy.ServiceProxy(
            'ebot_mani/set_pose_wrist', SetPose)
        self.align_wrist = rospy.ServiceProxy('ebot_mani/align_wrist', Empty)
        self.set_pose_wrist_no_align = rospy.ServiceProxy(
            'ebot_mani/set_pose_wrist_no_align', SetPose)

    def go_to_pose_wrist(self, arg_pose):
        req = SetPoseRequest()
        req.pose = arg_pose
        return self.set_pose_wrist(req).success

    def go_to_pose_wrist_no_align(self, arg_pose):
        req = SetPoseRequest()
        req.pose = arg_pose
        return self.set_pose_wrist_no_align(req).success

    def go_to_pose_relative(self, arg_pose):
        req = SetPoseRequest()
        req.pose = arg_pose
        return self.set_pose_relative(req).success

    # def graspObjectHorizontal(self, point, width, yaw=0):
    #     req = GraspObjectRequest()
    #     req.point = point
    #     req.width = width
    #     req.yaw = yaw
    #     return self.graspObjectHorizontalService(req)

    def graspObjectVerticalOld(self, point, width, yaw):
        req = GraspObjectRequest()
        req.point = point
        req.width = width
        req.yaw = yaw
        return self.graspObjectVerticalService(req).success

    def graspObjectVertical(self, point, width, yaw):
        '''
        Given the position of object within reach it grasps it.
        Argument : position (Point msg)
        '''
        self.align_wrist()
        req = GetTransformPointRequest()
        req.point = point
        req.from_frame = "base_link"
        req.to_frame = "wrist_3_link"
        point = transformPoint(req).point

        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0.25 * sin(yaw)
        graspPose.position.y -= 0.15  # + 0.1
        graspPose.position.z -= 0.12  # Should be 0.25 * sin(grasp_angle)

        # Pose just Above the object
        flag = self.go_to_pose_wrist(graspPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        # Set grasping angle
        if yaw != 0.0:
            newOPose = Pose()
            newOPose.orientation = orient_from_euler(0, 0, yaw)
            flag = self.go_to_pose_wrist(newOPose)

            if flag == False:
                rospy.logerr("Could not Reach grasp Pose")
                return False

        newOPose = Pose()
        newOPose.orientation = orient_from_euler(0.558505, 0, 0)  # 32 deg
        flag = self.go_to_pose_wrist_no_align(newOPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        newOPose = Pose()
        newOPose.position.z += 0.01
        flag = self.go_to_pose_wrist(newOPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        return flag

    def graspObjectHorizontal(self, point, width, yaw):
        '''
        Given the position of object within reach it grasps it.
        Argument : position (Point msg)
        '''
        self.align_wrist()
        req = GetTransformPointRequest()
        req.point = point
        req.from_frame = "base_link"
        req.to_frame = "wrist_3_link"
        point = transformPoint(req).point

        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0.25 * sin(yaw)
        graspPose.position.y -= 0.188  # + 0.1
        graspPose.position.z -= 0.07  # Should be 0.25 * sin(grasp_angle)

        # Pose just Above the object
        flag = self.go_to_pose_wrist(graspPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        # Set grasping angle
        if yaw != 0.0:
            newOPose = Pose()
            newOPose.orientation = orient_from_euler(0, 0, yaw)  # 32 deg
            flag = self.go_to_pose_wrist(newOPose)

            if flag == False:
                rospy.logerr("Could not Reach grasp Pose")
                return False

        newOPose = Pose()
        newOPose.orientation = orient_from_euler(0.558505, 0, 0)  # 32 deg
        flag = self.go_to_pose_wrist_no_align(newOPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        # # #Grasp
        self.closeGripper(width)
        rospy.sleep(1)

        newOPose = Pose()
        newOPose.position.z = -0.09
        flag = self.go_to_pose_wrist_no_align(newOPose)
        if flag == False:
            rospy.logerr("Could not Reach grasp Pose")
            return False

        return True

def main():
    # maind()
    getFPGA()
    
    ur5.openGripper()

def maind():
    ur5.go_to_named_pose("navPose")
    # ebot.go_to_goal('store_table_fpga')
    ebot.go_to_goal('store_table')
    # ebot.go_to_goal_precise('store_table')
    ebot.print_current_pose()
    # detect.detectTable()

    # ur5.go_to_named_pose("seeObjectJ")

    ur5.go_to_named_pose("fpgaPoseOdom")
    detect.detect()
    object_name = 'FPGA_board'
    
    pointBaseLink = detect.dict[object_name]
    graspPose_pub.publish(createPoseStamped(pointBaseLink))

    pointOdom = TransformPoint(pointBaseLink, 'base_link', 'odom')
    
    ur5.go_to_named_pose("graspVerticalJ")
    pose = Pose()
    pose.position.z = 0.1
    ur5.go_to_pose_relative(pose)

    ebot.go_to_goal_precise('store_table_close')
    ebot.go_to_waypoint_relative(0.4, 0 ,0)

    pointBaseLink = TransformPoint(pointOdom,'odom', 'base_link')
    graspPose_pub.publish(createPoseStamped(pointBaseLink))
    detect.detectTable()
    rospy.sleep(0.1)

    flag = ur5.graspObjectVerticalOld(
        pointBaseLink, width=w_dict[object_name], yaw=pi/3)
    while flag == False:
        ebot.go_to_waypoint_relative(0.2, 0, 0)
        detect.detect()
        flag = ur5.graspObjectVerticalOld(
            pointBaseLink, width=w_dict[object_name], yaw=pi/3)


    ur5.openGripper()

def getFPGAnew():

    ur5.go_to_named_pose("navPose")
    # ebot.go_to_goal('store_table_fpga')
    ebot.go_to_goal('store_table')
    ebot.go_to_goal_precise('store_table_fpga')
    ebot.print_current_pose()
    detect.detectTable()

    ur5.go_to_named_pose("seeObjectJ")

    ur5.go_to_named_pose("fpgaPoseOdom")
    ebot.go_to_waypoint_relative(0.25, 0, 0)
    ur5.go_to_named_pose("fpgaPoseOdom")

    detect.detect()
    detect.detectTable()

    ur5.openGripper()
    object_name = 'FPGA_board'
    graspPose_pub.publish(createPoseStamped(detect.dict[object_name]))
    printPoint(detect.dict[object_name])

    ur5.go_to_named_pose("graspVerticalJ")
    pose = Pose()
    pose.position.z = 0.1
    ur5.go_to_pose_relative(pose)

    flag = ur5.graspObjectVerticalOld(
        detect.dict[object_name], width=w_dict[object_name], yaw=pi/3)
    while flag == False:
        ebot.go_to_waypoint_relative(0.2, 0, 0)
        detect.detect()
        flag = ur5.graspObjectVerticalOld(
            detect.dict[object_name], width=w_dict[object_name], yaw=pi/3)

    ebot.go_to_pose_relative(-1, 0, 0, rospy.Duration(5))
    ur5.go_to_named_pose("navPose")

    ebot.go_to_goal("store_exit")

def getFPGA():

    ur5.go_to_named_pose("navPose")
    # ebot.go_to_goal('store_table_fpga')
    ebot.go_to_goal('store_table')
    ebot.go_to_goal_precise('store_table_fpga')
    ebot.print_current_pose()
    detect.detectTable()

    ur5.go_to_named_pose("seeObjectJ")

    ur5.go_to_named_pose("fpgaPoseOdom")
    ebot.go_to_waypoint_relative(0.25, 0, 0)
    ur5.go_to_named_pose("fpgaPoseOdom")

    detect.detect()
    detect.detectTable()

    ur5.openGripper()
    object_name = 'FPGA_board'
    graspPose_pub.publish(createPoseStamped(detect.dict[object_name]))
    printPoint(detect.dict[object_name])

    ur5.go_to_named_pose("seeObjectJ")

    flag = ur5.graspObjectVerticalOld(
        detect.dict[object_name], width=w_dict[object_name], yaw=pi/3)
    while flag == False:
        ebot.go_to_waypoint_relative(0.2, 0, 0)
        detect.detect()
        flag = ur5.graspObjectVerticalOld(
            detect.dict[object_name], width=w_dict[object_name], yaw=pi/3)

    ebot.go_to_pose_relative(-1, 0, 0, rospy.Duration(5))
    ur5.go_to_named_pose("navPose")

    ebot.go_to_goal("store_exit")
    
def getGlue():
    ur5.go_to_named_pose("navPose")

    # TODO check if in meeting Room
    # ebot.go_to_goal('meeting_entry')
    # print("Entered room")
    ebot.print_current_pose()
    ebot.go_to_goal_precise('meeting_table')
    ebot.go_to_goal('meeting_table')
    print("Reached Goal")
    ebot.print_current_pose()

    ebot.applyBrakes()
    detect.detectTable()
    ur5.go_to_named_pose("meetingTable")
    detect.detect()
    pickupObject('glue')
    ur5.go_to_named_pose("navPose")
    ebot.releaseBrakes()


def enter_pantry():
    ur5.go_to_named_pose("navPose")
    ebot.go_to_goal('pantry_entry')
    ebot.go_to_waypoint_relative(1.3, 0, 0)
    printReached("pantry")


def getCoke():

    enter_pantry()

    ebot.go_to_goal_precise('pantry_table1')
    ebot.go_to_goal('pantry_table1')

    ebot.applyBrakes()
    detect.detectTable()
    ur5.go_to_named_pose("pantryTable1Odom")

    detect.detect()
    pickupObject('coke_can')
    ur5.go_to_named_pose("navPoseOld")
    ebot.releaseBrakes()

    exit_pantry()


def exit_pantry():
    # ebot.go_to_goal('pantry_exit')
    # ebot.go_to_waypoint_relative(1.2,0,0)
    # ebot.go_to_goal('pantry_exit_old')

    ebot.go_to_goal_precise('pantry_exit')
    ebot.set_yaw(pi/2)
    ebot.go_to_waypoint_relative(1.2, 0, 0)


def dropbox3():
    ebot.go_to_goal('research_entry')
    ebot.print_current_pose()
    ebot.go_to_goal('research_dropbox')
    ebot.print_current_pose()
    ebot.applyBrakes()

    detect.detectTable()
    ur5.go_to_named_pose("researchDropbox")
    ur5.openGripper()
    rospy.sleep(0.5)
    ur5.go_to_named_pose("navPose")
    ebot.releaseBrakes()


def exit_meeting():
    ebot.go_to_goal_precise('meeting_exit')
    ebot.go_to_goal('meeting_exit')


def enter_meeting():
    ebot.go_to_goal('meeting_entry')
    ebot.go_to_waypoint_relative(1, 0, 0)


def dropbox2():

    ebot.go_to_goal_precise('meeting_dropbox')
    # ebot.go_to_goal('meeting_dropbox')
    ebot.print_current_pose()

    detect.detectTable()
    ur5.go_to_named_pose("researchDropboxJ")
    ur5.go_to_named_pose("meetingDropboxOdom")
    ur5.openGripper()
    rospy.sleep(0.5)
    ur5.go_to_named_pose("navPose")

    # ebot.go_to_pose_relative(0.95,0,0)


def enter_conference_room():
    ebot.go_to_goal('conference_entry')
    ebot.go_to_waypoint_relative(1, 0, 0)


def dropbox1():
    ur5.go_to_named_pose("navPose")
    enter_conference_room()
    ebot.go_to_goal('conference_dropbox')
    ebot.print_current_pose()
    ebot.applyBrakes()

    detect.detectTable()
    ur5.go_to_named_pose("conferenceDropbox")
    ur5.openGripper()
    rospy.sleep(0.5)
    ur5.go_to_named_pose("navPose")
    ebot.releaseBrakes()

    exit_conference_room()


def exit_conference_room():
    ebot.set_yaw(-3*pi/2)
    ebot.go_to_waypoint_relative(1, 0, 0)


def subtask1():
    getFPGA()
    dropbox1()


def subtask2():
    getCoke()
    enter_meeting()
    dropbox2()


def subtask3():

    getGlue()
    exit_meeting()
    dropbox3()


if __name__ == '__main__':

    rospy.init_node('grasping_node')

    graspPose_pub = rospy.Publisher("/graspPose", PoseStamped, queue_size=1)

    ur5 = Ur5()
    ebot = Ebot()
    detect = Detect()

    # main()
    getFPGA()
    # subtask1()
    # subtask2()
    # subtask3()
   

    # ebot.releaseBrakes()
