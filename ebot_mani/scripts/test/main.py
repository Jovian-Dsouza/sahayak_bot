#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
from geometry_msgs.msg import Point, Pose

def create_pose(x, y, z, qx, qy , qz, qw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = geometry_msgs.msg.Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('grasping_node', anonymous=True)

        self.arm_planning_group = "ur5_planning_group"
        self.gripper_planning_group = "gripper_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_planning_group)
        self.gripper_group = moveit_commander.MoveGroupCommander(self.gripper_planning_group)
        
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        #self.arm_group.set_pose_reference_frame("base_link")
        self._planning_frame = self.arm_group.get_planning_frame()
        self._eef_link = self.arm_group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()

        
        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self.arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self.arm_group.set_pose_target(arg_pose)
        flag_plan = self.arm_group.go(wait=True)  # wait=False for Async Move

        pose_values = self.arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self.arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan
    
    def set_joint_angles(self, arg_list_joint_angles):

        self.arm_group.set_joint_value_target(arg_list_joint_angles)
        self.arm_group.plan()
        flag_plan = self.arm_group.go(wait=True)

        list_joint_values = self.arm_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self.arm_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        #rospy.loginfo(pose_values)
        rospy.loginfo("(%f , %f, %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        pose_values.orientation.x,
                                                        pose_values.orientation.y,
                                                        pose_values.orientation.z,
                                                        pose_values.orientation.w))

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def set_gripper_joint(self, arg_list_joint_angles):

        list_joint_values = self.gripper_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self.gripper_group.set_joint_value_target(arg_list_joint_angles)
        self.gripper_group.plan()
        flag_plan = self.gripper_group.go(wait=True)

        list_joint_values = self.gripper_group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self.gripper_group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
      
        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def closeGripper(self, angle):
        # joint value Range : [0, 0.84]
        self.set_gripper_joint([angle])
        rospy.loginfo('\033[94m' + "Gripper Closed" + '\033[0m')

    def openGripper(self):
        self.set_gripper_joint([0.0])
        rospy.loginfo('\033[94m' + "Gripper Opened" + '\033[0m')

    def pickObject(self, poses, gripper_angle):
        self.go_to_pose(poses[0]) #Gripper above the object
        self.go_to_pose(poses[1]) #Gripper goes near to the object for grasping
        self.closeGripper(gripper_angle) #Grasps the object
        rospy.sleep(1.5)
        self.go_to_pose(poses[0]) #lift the object
        self.go_to_pose(poses[2]) #Gripper goes above the bin
        self.openGripper() #Drop the object

    def homePose(self):
        #home_joint_angles = [0.004061, -0.094609, 0.060309, -0.355868, 0.004787, -0.132758]
        home_joint_angles = [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ]
        self.set_joint_angles(home_joint_angles)

    def graspObject(self, position):
        '''
        Given the position of object within reach it grasps it.

        Argument : position (Point msg)
        Returns : True / False based if grasp was successful or not
        '''
        graspPose = Pose()
        graspPose.position = position
        graspPose.orientation = self.arm_group.get_current_pose().pose.orientation
        graspPose.orientation.x = -0.000294
        graspPose.orientation.y = -1.000000
        graspPose.orientation.z = 0.000314
        graspPose.orientation.w = 0.000202
        self.go_to_pose(graspPose)

        

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    ur5.homePose()

    rospy.sleep(1)
    del ur5


if __name__ == '__main__':
    main()

