#! /usr/bin/env python

# This helper module defines Transformer class which is used to transfom Point 
# or Pose object to different frames 

import tf2_ros
import tf2_geometry_msgs 
import rospy

class Transformer:
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

    def transform_pose(self, input_pose, from_frame, to_frame):

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = input_pose
        pose_stamped.header.frame_id = from_frame
        pose_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, to_frame, rospy.Duration(1))
            return output_pose_stamped.pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise

    def transform_point(self, input_point, from_frame, to_frame):

        point_stamped = tf2_geometry_msgs.PointStamped()
        point_stamped.point = input_point
        point_stamped.header.frame_id = from_frame
        point_stamped.header.stamp = rospy.Time.now()

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_point_stamped = self.tf_buffer.transform(point_stamped, to_frame, rospy.Duration(1))
            return output_point_stamped.point

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            raise