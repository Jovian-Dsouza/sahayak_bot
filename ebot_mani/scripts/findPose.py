#! /usr/bin/env python

import rospy
from ebot_mani.srv import *
from std_srvs.srv import Empty
from ebot_mani.mani_poses import *

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
        self.print_name_pose = rospy.ServiceProxy('ebot_mani/print_name_pose', SetNamedPose)
        self.getCurrentPoseOdom = rospy.ServiceProxy('ebot_mani/get_current_pose_odom', GetPose)
        
    def printCurrentPoseOdom(self):
        pose_values = self.getCurrentPoseOdom().pose

        rospy.loginfo('\033[94m' + ">>> Current Pose (Odom):" + '\033[0m')
        rospy.loginfo("(%f , %f, %f, %f, %f, %f, %f)" % (pose_values.position.x,
                                                        pose_values.position.y,
                                                        pose_values.position.z,
                                                        pose_values.orientation.x,
                                                        pose_values.orientation.y,
                                                        pose_values.orientation.z,
                                                        pose_values.orientation.w))

if __name__ == '__main__':
    rospy.init_node("findPose")
    detectTable = rospy.ServiceProxy('/ebot/detectTable', Empty)

    ur5 = Ur5()
    detectTable()
    ur5.printCurrentPoseOdom()
    ur5.prin
    print(ur5.go_to_pose(poses_dict["storeTable"]))
    
    

