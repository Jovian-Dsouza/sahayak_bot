#! /usr/bin/env python

#This helper module defines the basic Ur5Moveit Class

import rospy

import sys
import moveit_commander
import moveit_msgs.msg
import actionlib

from mani_poses import *
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import pi
from geometry_msgs.msg import Pose,Point, Quaternion, PoseStamped

from perception.srv import *
transformPose = rospy.ServiceProxy('/get_transform_pose', GetTransformPose)
transformPoint = rospy.ServiceProxy('/get_transform_point', GetTransformPoint)

def create_pose_quaternion(x, y, z ,qx ,qy ,qz ,qw):
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

def create_pose_euler(x, y, z, roll, pitch, yaw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = quaternion_from_euler(roll, pitch , yaw)

    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose



def euler_from_orient(o):
    '''
    Input is Quaternion pose.orientation
    output is roll, pitch, yaw
    '''
    a = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return (a[0], a[1], a[2])

def orient_from_euler(roll, pitch, yaw):
    '''
    Input is roll, pitch, yaw
    output is Quaternion pose.orientation
    '''
    q = quaternion_from_euler(roll, pitch, yaw)
    o = Quaternion()
    o.x, o.y, o.z, o.w = q[0], q[1], q[2], q[3]
    return o


class Ur5Moveit:
    def createPoseStamped(self, pose):
        poseStamped = PoseStamped()
        poseStamped.header.frame_id = self.arm_group.get_pose_reference_frame() #'base_link'
        poseStamped.header.stamp = rospy.Time.now()
        poseStamped.pose = pose
        return poseStamped

    # Constructor
    def __init__(self):
        
        self.arm_planning_group = "ur5_planning_group"
        self.gripper_planning_group = "gripper_planning_group"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_planning_group)
        
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
             moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self.arm_group.set_pose_reference_frame("base_link")

        self.goal_pose_pub = rospy.Publisher("/goal_pose", PoseStamped, queue_size=1)
        
        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def printInfo(self):
        ''' Prints info regrading planing parameters '''
        self._planning_frame = self.arm_group.get_planning_frame()
        self._eef_link = self.arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        rospy.loginfo(
            '\033[94m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

    def printCurrentPose(self):
        pose_values = self.arm_group.get_current_pose().pose

        #TODO need a better way
        #rosrun tf tf_echo ebot_base base_link
        #here am just doing a linear translation from
        #ebot_base -> base_link
        pose_values.position.z -= 0.521

        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo("(%f , %f, %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        pose_values.orientation.x,
                                                        pose_values.orientation.y,
                                                        pose_values.orientation.z,
                                                        pose_values.orientation.w))

    def printCurrentPoseEuler(self):
        pose_values = self.arm_group.get_current_pose().pose

        #TODO need a better way
        #rosrun tf tf_echo ebot_base base_link
        #here am just doing a linear translation from
        #ebot_base -> base_link
        pose_values.position.z -= 0.521
        
        o = pose_values.orientation
        a = euler_from_quaternion([o.x, o.y, o.z, o.w])
        rospy.loginfo('\033[94m' + ">>> Current Pose (in Euler):" + '\033[0m')
        rospy.loginfo("(%f , %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        a[0], a[1], a[2]))
    
    def printCurrentPoseEulerOdom(self):
        pose_values = self.arm_group.get_current_pose().pose
        #ebot_base -> odom
        req = GetTransformPoseRequest()
        req.pose = pose_values
        req.from_frame = "ebot_base"
        req.to_frame = "odom"
        resp = transformPose(req)
        pose_values = resp.pose
        
        o = pose_values.orientation
        a = euler_from_quaternion([o.x, o.y, o.z, o.w])
        rospy.loginfo('\033[94m' + ">>> Current Pose ODOM (in Euler):" + '\033[0m')
        rospy.loginfo("(%f , %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        a[0], a[1], a[2]))
    def getCurrentPoseOdom(self):
        #ebot_base -> odom
        req = GetTransformPoseRequest()
        req.pose = self.arm_group.get_current_pose().pose
        req.from_frame = "ebot_base"
        req.to_frame = "odom"
        return transformPose(req).pose


    def printCurrentJointValues(self):
        j = self.arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo("\033[94m[%f, %f, %f, %f, %f, %f]\033[0m" % (j[0], j[1], j[2], j[3], j[4], j[5]))

    # def go_to_pose(self, arg_pose):
    #     self.arm_group.set_pose_target(arg_pose)
    #     flag_plan = self.arm_group.go(wait=True)  # wait=False for Async Move

    #     if (flag_plan == True):
    #         rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
    #     else:
    #         rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

    #     return flag_plan

    def go_to_pose(self, arg_pose):
        try_count = 0
        self.goal_pose_pub.publish(self.createPoseStamped(arg_pose))
        self.arm_group.set_pose_target(arg_pose)
        flag_plan = self.arm_group.go(wait=True)  # wait=False for Async Move

        while(flag_plan == False and try_count < 5):
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed : " + str(try_count) + '\033[0m')
            try_count += 1
            flag_plan = self.arm_group.go(wait=True)
            rospy.sleep(0.1)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    def go_to_pose_odom(self, arg_pose):
        self.arm_group.set_pose_reference_frame("odom")
        flag = self.go_to_pose(arg_pose)
        self.arm_group.set_pose_reference_frame("base_link")
        return flag

    def go_to_pose_relative(self, arg_pose):
        #ebot_base -> odom
        req = GetTransformPoseRequest()
        req.pose = self.arm_group.get_current_pose().pose
        req.from_frame = "ebot_base"
        req.to_frame = "odom"
        resp = transformPose(req)
        p = resp.pose.position #current position
        o = resp.pose.orientation #current orientation

        arg_pose.position.x += p.x
        arg_pose.position.y += p.y
        arg_pose.position.z += p.z
        arg_pose.orientation = o

        self.go_to_pose_odom(arg_pose)
    
    def align_wrist(self):
        #curr_pose in ebot_base
        curr_pose = self.arm_group.get_current_pose().pose
        
        #save current roll pitch
        roll, pitch, yaw = euler_from_orient(curr_pose.orientation)

        #align wrist_3_link frame
        #set roll tp -pi, pitch to 0
        curr_pose.orientation = orient_from_euler(-pi, 0, yaw)
        self.arm_group.set_pose_reference_frame("ebot_base")
        self.go_to_pose(curr_pose)
        self.arm_group.set_pose_reference_frame("base_link")

    
    def go_to_pose_wrist(self, arg_pose):

        #curr_pose in ebot_base
        curr_pose = self.arm_group.get_current_pose().pose
        
        #save current roll pitch
        roll, pitch, yaw = euler_from_orient(curr_pose.orientation)

        #align wrist_3_link frame
        #set roll tp -pi, pitch to 0
        curr_pose.orientation = orient_from_euler(-pi, 0, yaw)
        self.arm_group.set_pose_reference_frame("ebot_base")
        self.go_to_pose(curr_pose)

        #Transform arg_pose to ebot_base
        req = GetTransformPoseRequest()
        req.pose = arg_pose
        req.from_frame = "wrist_3_link"
        req.to_frame = "ebot_base"
        arg_pose.position = transformPose(req).pose.position
        rospy.loginfo("In ebot_base ")
        print(arg_pose.position)

        a_roll, a_pitch, a_yaw = euler_from_orient(arg_pose.orientation)

        arg_pose.orientation = orient_from_euler(roll+a_roll, pitch+a_pitch, yaw+a_yaw)

        flag = self.go_to_pose(arg_pose)

        self.arm_group.set_pose_reference_frame("base_link")

        return flag

    def go_to_pose_wrist_no_align(self, arg_pose):

        #curr_pose in ebot_base
        curr_pose = self.arm_group.get_current_pose().pose
        
        #save current roll pitch
        roll, pitch, yaw = euler_from_orient(curr_pose.orientation)

        #Transform arg_pose to ebot_base
        req = GetTransformPoseRequest()
        req.pose = arg_pose
        req.from_frame = "wrist_3_link"
        req.to_frame = "ebot_base"
        arg_pose.position = transformPose(req).pose.position
        rospy.loginfo("In ebot_base ")
        print(arg_pose.position)

        a_roll, a_pitch, a_yaw = euler_from_orient(arg_pose.orientation)

        arg_pose.orientation = orient_from_euler(roll+a_roll, pitch+a_pitch, yaw+a_yaw)
        self.arm_group.set_pose_reference_frame("ebot_base")
        flag = self.go_to_pose(arg_pose)
        self.arm_group.set_pose_reference_frame("base_link")

        return flag

       

    
    def set_joint_angles(self, arg_list_joint_angles):
        self.arm_group.set_joint_value_target(arg_list_joint_angles)
        self.arm_group.plan()
        flag_plan = self.arm_group.go(wait=True)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def go_to_named_pose(self, name):
        
        if name in joint_angles_dict:
            if(self.set_joint_angles(joint_angles_dict[name])):
                rospy.loginfo('\033[94m' + " >>> Name Pose Reached : " + name + '\033[0m')
                return True
            else:
                return False
        
        elif name in poses_dict:
            if(self.go_to_pose(poses_dict[name])):
                rospy.loginfo('\033[94m' + " >>> Name Pose Reached : " + name + '\033[0m')
                return True
            else:
                return False

        elif name in odom_poses_dict:
            if(self.go_to_pose_odom(odom_poses_dict[name])):
                rospy.loginfo('\033[94m' + " >>> Name Pose Reached : " + name + '\033[0m')
                return True
            else:
                return False
        
        else:
            rospy.logerr(name + " Pose not found")
            return False

    def joint_angles_to_pose(self, list_joint_angles):
        flag = self.set_joint_angles(list_joint_angles)
        if(flag):
            self.printCurrentPoseEuler()
    
    def pose_to_joint_angles(self, arg_pose):
        flag = self.go_to_pose(arg_pose)
        if(flag):
            self.printCurrentJointValues()
    
    def name_pose_to_pose(self, name):
        flag = self.go_to_named_pose(name)
        if(flag):
            self.printCurrentPoseEuler()
    
    def name_pose_to_joint_angles(self, name):
        flag = self.go_to_named_pose(name)
        if(flag):
            self.printCurrentJointValues()
    
    def print_name_pose(self, name):
        '''Goes to the given name pose and 
            prints the current joint values and pose
        '''
        flag = self.go_to_named_pose(name)
        if(flag):
            self.printCurrentJointValues()
            self.printCurrentPoseEuler()
            self.printCurrentPoseEulerOdom()
        return flag

    def set_gripper_value(self, arg_list_joint_angles):
        self.gripper_group.set_joint_value_target(arg_list_joint_angles)
        self.gripper_group.plan()
        flag_plan = self.gripper_group.go(wait=True)
      
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')
        return flag_plan

    def closeGripper(self, angle):
        '''Angle Range : [0, 0.84)'''
        self.set_gripper_value([angle])
        rospy.loginfo('\033[94m' + "Gripper Closed" + '\033[0m')
    
    def closeGripperFull(self):
        self.set_gripper_value([0.8])
        rospy.loginfo('\033[94m' + "Gripper Closed" + '\033[0m')

    def openGripper(self):
        self.set_gripper_value([0.0])
        rospy.loginfo('\033[94m' + "Gripper Opened" + '\033[0m')

    ###############Grasping##################################
    def cal_grasp_quaternion(self, roll, yaw):
        #calculate the current yaw angle
        co = self.arm_group.get_current_pose().pose.orientation
        a = euler_from_quaternion([co.x, co.y, co.z, co.w])

        #Assuming that the gripper is pointing in the object direction
        q = quaternion_from_euler(-pi+roll, 0 , a[2] + yaw) 
        o = Quaternion()
        o.x , o.y, o.z, o.w = q[0], q[1], q[2], q[3]
        return o
        

    def graspObjectHorizontal(self, point, width, yaw):
        '''
        Given the position of object within reach it grasps it.
        Argument : position (Point msg)
        '''
        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0
        graspPose.position.y -= 0.183 + 0.1
        graspPose.position.z += 0.07#Should be 0.25 * sin(grasp_angle)
        graspPose.orientation = self.cal_grasp_quaternion(0.558505,yaw) #32 deg 
        
        #Front of object 
        flag = self.go_to_pose(graspPose)
        if(not flag):
            return False

        #Approach from y axis
        graspPose.position.y += 0.1
        self.go_to_pose(graspPose)
        
        #Grasp
        self.closeGripper(width) 
        rospy.sleep(1)

        #retreat from z axis
        graspPose.position.z += 0.2
        self.go_to_pose(graspPose)

        #TODO Detect when there is a failure
        return True

    def graspObjectVertical(self, point, width, yaw):
        '''
        Function used only for grasping object vertcally
        Takes in the position and the grasping angle as arguments
        '''
        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0
        graspPose.position.y += 0.0145

        #The yaw angle should be perpendicular to the objects angle
        graspPose.orientation = self.cal_grasp_quaternion(pi/2, pi/2 + yaw)
        
        #Above Object 
        #Distace between camera and gripper finger tip = 0.25
       
        graspPose.position.z += 0.248
        flag = self.go_to_pose(graspPose)
        if(not flag):
            return False
        self.printCurrentJointValues()
        
        h = 0.067
        # #Approach from z axis
        graspPose.position.z -= h
        self.go_to_pose(graspPose)
       
        
        # #Grasp
        self.closeGripper(width) 
        rospy.sleep(1)

        # #retreat from z axis
        graspPose.position.z += h
        self.go_to_pose(graspPose)

        #TODO Detect when there is a failure
        return True


    

    