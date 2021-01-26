#!/usr/bin/env python

'''
subscribes to /camera2/depth/points2
applies PCL filters (voxel , passthrough , RANSAC)
publishes to /pcl_objects and /pcl_table
'''

from pcl_helper import *
import rospy
from sensor_msgs.msg import PointCloud2 

def pcl_callback(pcl_msg):
    cloud = ros_to_pcl(pcl_msg)

    #Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.008
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # #PassThrough Filter y axis
    # passthrough = cloud_filtered.make_passthrough_filter()
    # filter_axis = 'y'
    # passthrough.set_filter_field_name(filter_axis)
    # axis_min = -0.2
    # axis_max = 1
    # passthrough.set_filter_limits(axis_min, axis_max)
    # cloud_filtered = passthrough.filter()

    #PassThrough Filter z axis
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0  #TODO Think of better way to set limits
    axis_max = 2
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    #RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    #Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    #Convert PCL data to ROS messages
    ros_cloud_objects = pcl_to_ros(cloud_objects) 
    ros_cloud_table = pcl_to_ros(cloud_table)

    #Publish ROS messages
    pcl_object_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)


if __name__ == '__main__':
    rospy.init_node('pcl_filter', anonymous=True)
    rospy.Subscriber('/camera2/depth/points2', PointCloud2, pcl_callback, queue_size=1)

    pcl_object_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()