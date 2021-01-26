#! /usr/bin/env python

#This helper module defines the basic Ur5Moveit Class

import rospy

import sys
import moveit_commander
import moveit_msgs.msg
import actionlib


class Ur5Moveit:

    # Constructor
    def __init__(self):
        
        self.arm_planning_group = "ur5_planning_group"
        self.gripper_planning_group = "gripper_planning_group"

        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_planning_group)
        
        self._display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
             moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self.arm_group.set_pose_reference_frame("base_link")
        
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
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo("(%f , %f, %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        pose_values.orientation.x,
                                                        pose_values.orientation.y,
                                                        pose_values.orientation.z,
                                                        pose_values.orientation.w))

    def printCurrentJointValues(self):
        j = self.arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo("\033[94m[%f, %f, %f, %f, %f, %f]\033[0m" % (j[0], j[1], j[2], j[3], j[4], j[5]))

    def go_to_pose(self, arg_pose):
        self.arm_group.set_pose_target(arg_pose)
        flag_plan = self.arm_group.go(wait=True)  # wait=False for Async Move

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr('\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
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

    