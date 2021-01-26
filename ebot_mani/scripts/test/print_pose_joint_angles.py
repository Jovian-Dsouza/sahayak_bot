#! /usr/bin/env python

import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg1_node_print_pose_joint_angles', anonymous=True)

        self._planning_group = "ur5_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def print_pose_ee(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        quaternion_list = [q_x, q_y, q_z, q_w]
        (roll, pitch, yaw) = euler_from_quaternion(quaternion_list)

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "roll: {}\n".format(roll) +
                      "pitch: {}\n".format(pitch) +
                      "yaw: {}\n".format(yaw) +
                      '\033[0m')
        
    def print_pose_ee_quat(self):
        pose_values = self._group.get_current_pose().pose

        # Convert Quaternion to Euler (Roll, Pitch, Yaw)
        q_x = pose_values.orientation.x
        q_y = pose_values.orientation.y
        q_z = pose_values.orientation.z
        q_w = pose_values.orientation.w

        rospy.loginfo('\033[94m' + "\n" + "End-Effector ({}) Pose: \n\n".format(self._eef_link) +
                      "x: {}\n".format(pose_values.position.x) +
                      "y: {}\n".format(pose_values.position.y) +
                      "z: {}\n\n".format(pose_values.position.z) +
                      "q_x: {}\n".format(q_x) +
                      "q_y: {}\n".format(q_y) +
                      "q_z: {}\n".format(q_z) +
                      "q_w: {}\n".format(q_w) +
                      '\033[0m')

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()
        j = list_joint_values

        # rospy.loginfo('\033[94m' + "\nJoint Values: In degrees \n\n" +
        #               "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
        #               "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
        #               "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
        #               "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
        #               "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
        #               "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
        #               '\033[0m')
        print("HERE ARE UR JOINT ANGLES")
        rospy.loginfo("\033[94m[%0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f]\033[0m" % (j[0], j[1], j[2], j[3], j[4], j[5]))


    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    # while not rospy.is_shutdown():
    #     ur5.print_pose_ee()
    #     ur5.print_joint_angles()
    #     rospy.sleep(1)

    ur5.print_joint_angles()
    rospy.sleep(1)

    del ur5


if __name__ == '__main__':
    main()