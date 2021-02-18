#!/usr/bin/env python

import rospy
import math
from math import pi

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# from ebot_mani.ur5_helper import Ur5Moveit
from poses import poses

from nav_msgs.msg import Odometry
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

from perception.srv import *
transformPose = rospy.ServiceProxy('/get_transform_pose', GetTransformPose)
transformPoint = rospy.ServiceProxy('/get_transform_point', GetTransformPoint)

def euler_from_orient(o):
    '''
    Input is Quaternion pose.orientation
    output is roll, pitch, yaw
    '''
    a = euler_from_quaternion([o.x, o.y, o.z, o.w])
    return (a[0], a[1], a[2])

def distance(x1 , x2 , y1 , y2):
    return math.sqrt((x1-x2)** 2 + (y1-y2)**2)

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    ebot.go_to_goal('pantry_entry')
    ebot.go_to_waypoint_relative(1.2,0,0)
    ebot.go_to_goal_precise('pantry_table1')
    ebot.go_to_goal('pantry_exit')
    ebot.go_to_waypoint_relative(1.2,0,0)

class Ebot():

    def __init__(self):
        rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server()
        rospy.loginfo("Connected to move base server")

        #Go to Goal Algorithm
        self.kp = 0.3
        self.yaw_prescision = math.pi * 5 / 180 # +- 5 degree
        self.dist_prescision = 0.13 #distace precision for the goal
        self.linear_velocity = 0.3 #const linear velocity
        self.angular_velocity = 0.4 #max velocity
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    def go_to_waypoint_relative(self, x, y , yaw_goal):
        '''
        Input Goal : geometry_msgs/Point    in ebot_base frame
        '''
        req = GetTransformPoseRequest()
        req.pose.position.x = x
        req.pose.position.y = y
        req.from_frame = "ebot_base"
        req.to_frame = "odom"
        goalPose = transformPose(req).pose 
        x = goalPose.position.x
        y = goalPose.position.y
        self.go_to_waypoint(x, y, yaw_goal)

    def go_to_waypoint(self, x, y , yaw_goal):
        '''
        Input Goal : geometry_msgs/Point    in Odom frame
        '''

        o = self.pose.orientation
        yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        pos = self.pose.position

        # yaw_goal += yaw

        velocity_msg = Twist()

        while(True):
            o = self.pose.orientation
            yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            pos = self.pose.position

            des_yaw = math.atan2(y-pos.y, x-pos.x)
            err_yaw = normalize_angle(des_yaw - yaw)
    
            if(math.fabs(err_yaw) > self.yaw_prescision):
                        velocity_msg.linear.x = 0.0
                        if(des_yaw >= yaw):
                            velocity_msg.angular.z = self.angular_velocity
                        else:
                            velocity_msg.angular.z = -self.angular_velocity
                    
            else: #waypoint reached
                rospy.loginfo("Waypoint angle set")
                self.vel_pub.publish(Twist()) #stop
                break
            
            self.vel_pub.publish(velocity_msg)
            rospy.sleep(0.05) 

        while(True):
            o = self.pose.orientation
            yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            pos = self.pose.position

            des_yaw = math.atan2(y-pos.y, x-pos.x)
            err_yaw = normalize_angle(des_yaw - yaw)
            err_dist = distance(x,pos.x, y, pos.y)

            if(math.fabs(err_dist) > self.dist_prescision):
                        velocity_msg.linear.x = self.linear_velocity
                        velocity_msg.angular.z = self.kp * err_yaw
            
            else: #waypoint reached
                self.vel_pub.publish(Twist()) #stop
                print("waypoint Reached")
                break
            
            self.vel_pub.publish(velocity_msg)
            rospy.sleep(0.05) 

    def set_yaw(self, yaw_goal):
        '''
        yaw goals are in odom frame
        '''
        velocity_msg = Twist()
        while(True):
            o = self.pose.orientation
            yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    
            des_yaw = yaw_goal
            err_yaw = normalize_angle(des_yaw - yaw)

            if(math.fabs(err_yaw) > self.yaw_prescision):
                        velocity_msg.linear.x = 0.0
                        if(des_yaw >= yaw):
                            velocity_msg.angular.z = self.angular_velocity
                        else:
                            velocity_msg.angular.z = -self.angular_velocity
            
            else:
                self.vel_pub.publish(Twist()) #stop
                break
            
            self.vel_pub.publish(velocity_msg)
            rospy.sleep(0.1)

    
    def go_to_waypoint_old(self, x, y , yaw_goal):
        '''
        Input Goal : geometry_msgs/Point    in Odom frame
        '''

        o = self.pose.orientation
        yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
        pos = self.pose.position

        yaw_goal += yaw

        velocity_msg = Twist()

        while(True):
            o = self.pose.orientation
            yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
            pos = self.pose.position

            des_yaw = math.atan2(y-pos.y, x-pos.x)
            err_yaw = normalize_angle(des_yaw - yaw)
            err_dist = distance(x,pos.x, y, pos.y)

            if(math.fabs(err_yaw) > self.yaw_prescision):
                        velocity_msg.linear.x = 0.0
                        if(des_yaw >= yaw):
                            velocity_msg.angular.z = self.angular_velocity
                        else:
                            velocity_msg.angular.z = -self.angular_velocity
                        # print(err_yaw)

            elif(math.fabs(err_dist) > self.dist_prescision):
                        velocity_msg.linear.x = self.linear_velocity
                        velocity_msg.angular.z = self.kp * (des_yaw - yaw)
            
            else: #waypoint reached
                self.vel_pub.publish(Twist()) #stop
                print("waypoint Reached")
                break
            
            self.vel_pub.publish(velocity_msg)
            rospy.sleep(0.1) 

        # while(True):
        #     o = self.pose.orientation
        #     yaw = euler_from_quaternion([o.x, o.y, o.z, o.w])[2]
    
        #     des_yaw = yaw_goal
        #     err_yaw = normalize_angle(des_yaw - yaw)

        #     if(math.fabs(err_yaw) > self.yaw_prescision):
        #                 velocity_msg.linear.x = 0.0
        #                 if(des_yaw >= yaw):
        #                     velocity_msg.angular.z = self.angular_velocity
        #                 else:
        #                     velocity_msg.angular.z = -self.angular_velocity
            
        #     else:
        #         self.vel_pub.publish(Twist()) #stop
        #         break
            
        #     self.vel_pub.publish(velocity_msg)
        #     rospy.sleep(0.1)
   

    def applyBrakes(self):
        pass
        # attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
        #                             Attach)
        # attach_srv.wait_for_service()
        # rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")
        # rospy.sleep(5)
        # # Link them
        # rospy.loginfo("Applying brakes to ebot")
        # req = AttachRequest()
        # req.model_name_1 = "ebot"
        # req.link_name_1 = "ebot_base"
        # req.model_name_2 = "ground_plane"
        # req.link_name_2 = "link"
        # attach_srv.call(req)
        

    def releaseBrakes(self):
        pass
        # rospy.loginfo("Creating ServiceProxy to /link_attacher_node/detach")
        # detach_srv = rospy.ServiceProxy('/link_attacher_node/detach',
        #                                 Attach)
        # detach_srv.wait_for_service()
        # rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        # # Link them
        # rospy.loginfo("Releasing Brakes")
        # req = AttachRequest()
        # req.model_name_1 = "ebot"
        # req.link_name_1 = "ebot_base"
        # req.model_name_2 = "ground_plane"
        # req.link_name_2 = "link"
        # detach_srv.call(req)

    def odom_cb(self, msg):
        self.pose = msg.pose.pose
        
    def print_current_pose(self):
        o = self.pose.orientation
        self.yaw = euler_from_quaternion([o.x , o.y, o.z , o.w])[2] # yaw
        self.p = self.pose.position
        print("(%0.5f, %0.5f, %0.5f)" % (self.p.x, self.p.y, self.yaw))

    def go_to_pose(self, goalPose, duration=rospy.Duration(60)):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        self.goal.target_pose.pose = goalPose
        rospy.loginfo("Sending goal pose to Action Server")
        self.client.send_goal_and_wait(self.goal, duration, duration)
        #self.client.send_goal(self.goal, self.done_cb, self.active_cb, self.feedback_cb)
        

    def go_to_pose_relative(self, x, y , theta, duration = rospy.Duration(60)):
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "ebot_base"
        self.goal.target_pose.header.stamp = rospy.Time.now() 
        self.goal.target_pose.pose.position.x = x
        self.goal.target_pose.pose.position.y = y
        self.goal.target_pose.pose.position.z = 0.0

        q = quaternion_from_euler(0,0, theta)
        self.goal.target_pose.pose.orientation.x = q[0]
        self.goal.target_pose.pose.orientation.y = q[1]
        self.goal.target_pose.pose.orientation.z = q[2]
        self.goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("Sending goal pose to Action Server")
        self.client.send_goal_and_wait(self.goal, duration, duration)

    def done_cb(self, status, result):
        if status == 2: #Cancel request
            rospy.loginfo("Goal pose received a cancel request after it started executing, completed execution!")
            return 

        if status == 3: #Goal Reached
            rospy.loginfo("Goal pose reached") 
            return

        if status == 4: #Goal was aborted
            rospy.loginfo("Goal pose was aborted by the Action Server")
            return

        if status == 5: #Rejected by action server
            rospy.loginfo("Goal pose has been rejected by the Action Server")
            return

        if status == 8:
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")

    def active_cb(self):
        rospy.loginfo("Goal pose is now being processed by the Action Server...")

    def feedback_cb(self, feedback):
        pass

    def go_to_goal(self, goal_name , duration=rospy.Duration(60)):
        start_time = rospy.Time.now()
        self.go_to_pose(poses[goal_name], duration)
        stop_time = rospy.Time.now()
        rospy.loginfo("Time Taken %0.4f" % (stop_time-start_time).to_sec())

    def go_to_goal_precise(self, goal_name):
        pose = poses[goal_name]
        _ , _ , yaw = euler_from_orient(pose.orientation)
        self.go_to_waypoint(pose.position.x, pose.position.y, yaw)
        

def create_2d_pose(x, y , theta):
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = 0.0

    q = quaternion_from_euler(0,0, theta)
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def enter_meeting_room():
    ebot.go_to_goal('meeting_entry_new')
    ebot.go_to_waypoint_relative(1.1, 0, 0)

def enter_conference_room():
    ebot.go_to_goal('conference_entry')
    ebot.go_to_waypoint_relative(1, 0, 0)

def exit_conference_room():
    ebot.set_yaw(-3*pi/2)
    ebot.go_to_waypoint_relative(1, 0, 0)

if __name__ == '__main__':
    rospy.init_node('NavTest')
    ebot = Ebot()
  
    # enter_conference_room()
    # ebot.go_to_goal('conference_dropbox')
    exit_conference_room()