#!/usr/bin/env python

from pcl_helper import *
from transform_helper import Transformer
from image_helper import ImagePub
import pcl

import rospy
from sensor_msgs.msg import PointCloud2 
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
import numpy as np
from std_srvs.srv import Empty, EmptyResponse
from ebot_mani.srv import AddPlane, AddPlaneRequest

#SVM
from svmClassifier import Classifier

camera_matrix = np.array([[554.3827128226441, 0.0, 320.5, 0],\
            [0.0, 554.3827128226441, 240.5, 0.0],\
            [0.0, 0.0, 1.0, 0.0]])

def euclidiean_cluster(cloud_objects):
    #Remove RGB data from PCL cloud
    white_cloud = pcl.PointCloud()
    points = []
    for p in cloud_objects:
        points.append([p[0], p[1], p[2]])
    white_cloud.from_list(points)
    
    tree = white_cloud.make_kdtree()

    # Create a cluster extraction object
    ec = white_cloud.make_EuclideanClusterExtraction()
    ec.set_ClusterTolerance(0.015)
    ec.set_MinClusterSize(25)
    ec.set_MaxClusterSize(2000)
    ec.set_SearchMethod(tree) # Search the k-d tree for clusters

    # Extract indices for each of the discovered clusters
    return ec.Extract()

def mapToImage(x , y, z):
    point_3d = np.array([(x, y, z, 1.0)])
    point_2d = np.matmul(camera_matrix, point_3d.T)
    x = int(np.asscalar(point_2d[0] / point_2d[2]))
    y = int(np.asscalar(point_2d[1] / point_2d[2]))
    return x, y

def numpyToPoint(np_array):
    p = Point()
    p.x = np.asscalar(np_array[0])
    p.y = np.asscalar(np_array[1])
    p.z = np.asscalar(np_array[2])
    return p

class DetectObject:
    def __init__(self):
        rospy.Subscriber('/camera2/depth/points2', PointCloud2, self.callback, queue_size=1)
        self.detection_info_pub = rospy.Publisher("/detection_info", ObjectPose, latch=True, queue_size=1)
        self.service = rospy.Service('/ebot/detect', Empty, self.detect)
        rospy.Service('/ebot/detectTable', Empty, self.detectTable_cb)

        self.pcl_object_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
        self.pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1, latch=True)

        self.tablePose_pub = rospy.Publisher("/table_Pose", PoseStamped, queue_size=1)
        self.addPlane = rospy.ServiceProxy('ebot_mani/add_plane', AddPlane)
        self.clear_octomap = rospy.ServiceProxy('/clear_octomap', Empty)
        
    def callback(self, msg): #/camera2/depth/points2
        self.ros_cloud = msg
    
    def pclFilter(self):
        cloud = ros_to_pcl(self.ros_cloud)

        #Voxel Grid Downsampling
        vox = cloud.make_voxel_grid_filter()
        LEAF_SIZE = 0.008
        vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
        cloud_filtered = vox.filter()

        # PassThrough Filter x axis
        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'x'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = -1
        axis_max = 1
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()

        # #PassThrough Filter y axis
        passthrough = cloud_filtered.make_passthrough_filter()
        filter_axis = 'y'
        passthrough.set_filter_field_name(filter_axis)
        axis_min = -0.2
        axis_max = 0.8
        passthrough.set_filter_limits(axis_min, axis_max)
        cloud_filtered = passthrough.filter()

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
        self.pcl_table = cloud_filtered.extract(inliers, negative=False)
        self.pcl_objects = cloud_filtered.extract(inliers, negative=True)

        #Convert PCL data to ROS messages
        ros_cloud_objects = pcl_to_ros(self.pcl_objects) 
        ros_cloud_table = pcl_to_ros(self.pcl_table)

        #Publish ROS messages
        self.pcl_object_pub.publish(ros_cloud_objects)
        self.pcl_table_pub.publish(ros_cloud_table)

    def detectTable(self):
        pcl_cluster_arr = self.pcl_table.to_array()
        centroid = np.mean(pcl_cluster_arr , axis=0)[:3]
        centroid_point = Point()
        centroid_point.x = np.asscalar(centroid[0])
        centroid_point.y = np.asscalar(centroid[1])
        centroid_point.z = np.asscalar(centroid[2])
        centroid_point = transformer.transform_point(centroid_point, 'camera_rgb_frame2' , 'base_link')
        

        tablePose = PoseStamped()
        tablePose.header.frame_id = 'base_link'
        tablePose.header.stamp = rospy.Time.now()
        tablePose.pose.position = centroid_point
        tablePose.pose.orientation.x = 0 
        tablePose.pose.orientation.y = -0.7071
        tablePose.pose.orientation.z = 0
        tablePose.pose.orientation.w = 0.7071

        # req = AddPlaneRequest()
        # req.name = "table"
        # req.pose = tablePose
        # self.addPlane(req)

        self.tablePose_pub.publish(tablePose)
        return centroid_point.z

    def detectTable_cb(self, req):
        self.clear_octomap()
        self.pclFilter()
        self.detectTable()
        return EmptyResponse()

    def detect(self, req):
        self.clear_octomap()
        self.pclFilter()
        tableHeight = self.detectTable()
        

        cloud_objects = self.pcl_objects
        cluster_indices = euclidiean_cluster(cloud_objects)

        imagePub.capture_image()

        objPoseMsg = ObjectPose()

        for index , pts_list in enumerate(cluster_indices):
            pcl_cluster = cloud_objects.extract(pts_list)
            pcl_cluster_arr = pcl_cluster.to_array()

            centroid = np.mean(pcl_cluster_arr , axis=0)[:3]

            grasp_point = Point()
            grasp_point.x = np.asscalar(centroid[0])
            grasp_point.y = np.asscalar(centroid[1])
            grasp_point.z = np.asscalar(centroid[2])

            #Transform the grasp_point to base_link 
            grasp_point = transformer.transform_point(grasp_point, 'camera_rgb_frame2' , 'base_link')

            #check if the centroid of object is above the table
            if(grasp_point.z < tableHeight):
                continue

            #Grasp point above the object 
            grasp_point.z = tableHeight + 2*(grasp_point.z-tableHeight)
            print(grasp_point.z)

            max_val = np.max(pcl_cluster_arr, axis=0)[:2]
            min_val = np.min(pcl_cluster_arr, axis=0)[:2]

            
            x1, y1 = mapToImage(min_val[0], min_val[1], centroid[2])
            x2, y2 = mapToImage(max_val[0], max_val[1], centroid[2])

            #Classifier
            label = classifier.predict(pcl_cluster)
            rospy.loginfo("DETECTED " + label)

            imagePub.draw_rectangle_with_label([x1, y1, x2, y2], label)
            imagePub.publish_image()

            objPoseMsg.name = label
            objPoseMsg.pose.header.frame_id = 'base_link'
            objPoseMsg.pose.header.stamp = rospy.Time.now()
            objPoseMsg.pose.pose.position = grasp_point

            #TODO REMOVE this berfore submission
            # width = np.asscalar(max_val[0] - min_val[0])
            # objPoseMsg.pose.pose.orientation.w = width
            # hight = np.asscalar(max_val[1] - min_val[1])
            # objPoseMsg.pose.pose.orientation.z = hight
    
            self.detection_info_pub.publish(objPoseMsg)
            rospy.sleep(0.1)

            #print("%d, %d, %d, %d %0.5f" % (x1, y1, x2, y2, width))

        return EmptyResponse()

if __name__ == '__main__':
    rospy.init_node('pclFilterAndobjDetection')
    transformer = Transformer()
    classifier = Classifier()
    objDetector = DetectObject()
    imagePub = ImagePub()
    rospy.loginfo("Started Object detection")
    rospy.spin()