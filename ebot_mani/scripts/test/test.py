#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import random
import math
from geometry_msgs.msg import Point, Pose , PointStamped
from std_msgs.msg import Header
import tf2_ros
import tf2_geometry_msgs 
from std_srvs.srv import Empty, EmptyRequest
from object_msgs.msg import ObjectPose
from tf.transformations import quaternion_from_euler, euler_from_quaternion

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
            '\033[94m' + "Planning Frame: {}".format(self._planning_frame) + '\033[0m')
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
        Returns : True / False based if grasp was successful or not
        '''
        graspPose = Pose()
        graspPose.position = point
        graspPose.position.x -= 0
        graspPose.position.y -= 0.183 + 0.1
        graspPose.position.z += 0.12
        # graspPose.orientation = self.arm_group.get_current_pose().pose.orientation
        angle = (math.pi/180) * 32
        q = quaternion_from_euler(-math.pi+angle, 0 , -math.pi)
        graspPose.orientation.x = q[0]
        graspPose.orientation.y = q[1]
        graspPose.orientation.z = q[2]
        graspPose.orientation.w = q[3]

        # graspPose.orientation.x = 0.008581
        # graspPose.orientation.y = 0.965395
        # graspPose.orientation.z = -0.258650
        # graspPose.orientation.w = 0.032232
        self.go_to_pose(graspPose)
        #rospy.sleep(10)
        graspPose.position.y += 0.1
        self.go_to_pose(graspPose)
        
        # rospy.sleep(2)
        self.closeGripper(width+0.2)
        rospy.sleep(1)

        graspPose.position.z += 0.2
        self.go_to_pose(graspPose)


        

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

def transform_pose(input_pose, from_frame, to_frame):

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = input_pose
    pose_stamped.header.frame_id = from_frame
    pose_stamped.header.stamp = rospy.Time.now()

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise


def main2():
    ur5 = Ur5Moveit()
  
    point_list1 = [0.03389, -0.08859, 0.56814, 0.07086] #coke
    point_list2 = [0.16338, 0.00068, 0.45650, 0.06000] #battery
    point_list3 = [-0.09098, -0.01799, 0.44693, 0.05859] #glue
    all_list = [point_list1, point_list2 , point_list3]

    w_list = []
    transformed_pose_list = []

    for point_list in all_list:
        grasp_pos = Pose()
        grasp_pos.position.x = point_list[0]
        grasp_pos.position.y = point_list[1]
        grasp_pos.position.z = point_list[2] 
        transformed_pose_list.append(transform_pose(grasp_pos, "camera_rgb_frame2", "ebot_base"))
        w_list.append(point_list[3])
    
    index = 2
    for index in range(3):
        w = w_list[index]
        transformed_pose = transformed_pose_list[index]
        ur5.graspObject(transformed_pose.position, width=w)
        rospy.sleep(1)
        ur5.goToDropBox()
        ur5.openGripper()
        ur5.homePose()
        rospy.sleep(1)

    del ur5

def main():
    ur5 = Ur5Moveit()
    
    pickup_list = ['coke_can', 'battery', 'glue']
    w_dict = {'coke_can': 0.07086, 'battery': 0.06000, 'glue' : 0.05859}
    transformed_pose_list = []
    w_list = []

    for model in pickup_list:
        grasp_pos = Pose()
        grasp_pos.position = detection_obj[model]
        transformed_pose_list.append(transform_pose(grasp_pos, "camera_rgb_frame2", "ebot_base"))
        w_list.append(w_dict[model])
    
    for index in range(3):
        w = w_list[index]
        transformed_pose = transformed_pose_list[index]
        ur5.graspObject(transformed_pose.position, width=w)
        rospy.sleep(1)
        ur5.goToDropBox()
        ur5.openGripper()
        ur5.homePose()
        rospy.sleep(1)

    del ur5

def angle_calibrate():
    ur5 = Ur5Moveit()
    ur5.homePose()
    new_pose = ur5.arm_group.get_current_pose().pose
    nq = new_pose.orientation
    print(euler_from_quaternion([nq.x, nq.y, nq.z, nq.w]))
    angle = (math.pi/180) * 90
    q = quaternion_from_euler(-math.pi+angle, 0 , -math.pi)
    new_pose.orientation.x = q[0]
    new_pose.orientation.y = q[1]
    new_pose.orientation.z = q[2]
    new_pose.orientation.w = q[3]
    ur5.go_to_pose(new_pose)

    new_pose = ur5.arm_group.get_current_pose().pose
    nq = new_pose.orientation
    print(euler_from_quaternion([nq.x, nq.y, nq.z, nq.w]))
    ur5.homePose()
    del ur5

def detect_callback(msg):
    detection_obj[msg.name] = msg.pose.pose.position
    rospy.loginfo(detection_obj)

if __name__ == '__main__':
    rospy.init_node('grasping_node', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    
    detection_obj = {}
    rospy.wait_for_service('/detect')
    detect = rospy.ServiceProxy('detect', Empty)
    rospy.Subscriber("/detection_info", ObjectPose, detect_callback)
    detect()
    #angle_calibrate()
    main()



#  def graspObject(self, point, width):
#         '''
#         Given the position of object within reach it grasps it.

#         Argument : position (Point msg)
#         Returns : True / False based if grasp was successful or not
#         '''
#         graspPose = Pose()
#         graspPose.position = point
#         graspPose.position.x -= 0.015
#         graspPose.position.y -= 0.185 + 0.1
#         graspPose.position.z += 0.12
#         # graspPose.orientation = self.arm_group.get_current_pose().pose.orientation

#         graspPose.orientation.x = 0.008581
#         graspPose.orientation.y = 0.965395
#         graspPose.orientation.z = -0.258650
#         graspPose.orientation.w = 0.032232
#         self.go_to_pose(graspPose)

#         graspPose.position.y += 0.1
#         self.go_to_pose(graspPose)
        
#         # rospy.sleep(2)
#         self.closeGripper(width+0.2)
#         rospy.sleep(1)


#         graspPose.position.z += 0.2
#         self.go_to_pose(graspPose)