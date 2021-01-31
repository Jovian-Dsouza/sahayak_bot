#! /usr/bin/env python

'''
This node uses the detection_info topic and performs the actual Ur5 arm manipulation
'''

import rospy
import random
import math

from geometry_msgs.msg import Point, Quaternion, Pose , PointStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
from std_srvs.srv import Empty

from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

from ebot_mani.ur5_helper import Ur5Moveit

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

class Ur5(Ur5Moveit):
    def __init__(self):
        Ur5Moveit.__init__(self)
        self.grasp_quaternion = roll_angle_to_quaternion(32)

    def homePose(self):
        home_joint_angles = [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ]
        self.set_joint_angles(home_joint_angles)
    
    def goToDropBox(self):
        # dropBoxPose = Pose()
        # dropBoxPose.position.y = 2.64  - 2.60130259295 - 0.15
        # dropBoxPose.position.x = 8.2  - 7.67449459357
        # dropBoxPose.position.z = 1.03
        # dropBoxPose.orientation = self.arm_group.get_current_pose().pose.orientation
        # self.go_to_pose(dropBoxPose)

        rand = random.uniform(-0.25, 0.25)
        angles = [-0.4133827614046668 + rand, 0.13464592230127082, -0.7849043025879245, -0.8762423747543746, -0.6320390724043712, 0.8254714018437523]
        self.set_joint_angles(angles)

    def graspObject(self, point, width):
        '''
        Given the position of object within reach it grasps it.
        Argument : position (Point msg)
        '''
        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0
        graspPose.position.y -= 0.183 + 0.1
        graspPose.position.z += 0.12
        graspPose.orientation = self.grasp_quaternion
        
        #Front of object 
        self.go_to_pose(graspPose)

        #Approach from y axis
        graspPose.position.y += 0.1
        self.go_to_pose(graspPose)
        
        #Grasp
        self.closeGripper(width) 
        rospy.sleep(1)

        #retreat from z axis
        graspPose.position.z += 0.2
        self.go_to_pose(graspPose)

    def graspEYIFI(self, point, yaw=90):
        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0
        graspPose.position.y += 0.0145
        graspPose.position.z += 0.12
        graspPose.orientation = angle_to_quaternion(90, yaw)
        
        #Front of object 
        graspPose.position.z += 0.2
        ur5.go_to_pose(graspPose)

        
        h = 0.07
        # #Approach from z axis
        graspPose.position.z -= h
        ur5.go_to_pose(graspPose)
        
        # #Grasp
        ur5.closeGripper(0.46) 
        rospy.sleep(1)

        # #retreat from z axis
        graspPose.position.z += h
        ur5.go_to_pose(graspPose)

def main():
    pickup_list = ['coke_can', 'battery', 'glue'] #Order in which to pickup items
    w_dict = {'coke_can': 0.27086, 'battery': 0.26500, 'glue' : 0.31}  # calcalated from x2 - x1 + 0.2
    transformed_pose_list = []
    w_list = []

    for model in pickup_list:
        grasp_point = Point()
        grasp_point = detect.dict[model]
        
        ur5.graspObject(grasp_point, width=w_dict[model])
        ur5.goToDropBox()
        ur5.openGripper()
        ur5.homePose()
        rospy.sleep(0.1)


if __name__ == '__main__':
    rospy.init_node('grasping_node')

    detect = Detect()
    detect.detect() #Call the detect service
    ur5 = Ur5()
    ur5.homePose()
    ur5.openGripper()
    ur5.graspEYIFI(detect.dict['eYFi_board'], 90+45)
    ur5.goToDropBox()
    ur5.openGripper()
    del ur5

