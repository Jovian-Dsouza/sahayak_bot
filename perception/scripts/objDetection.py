#!/usr/bin/env python

'''
This script starts an object detection service which uses the filtered pcl data 
to detect and recognize the object
'''


from pcl_helper import *
from transform_helper import Transformer
from image_helper import ImagePub
import pcl

import rospy
from sensor_msgs.msg import PointCloud2 
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from object_msgs.msg import ObjectPose
import numpy as np
from std_srvs.srv import Empty, EmptyResponse

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

class DetectObject:
    def __init__(self):
        rospy.Subscriber('/pcl_objects', PointCloud2, self.callback, queue_size=1)
        self.detection_info_pub = rospy.Publisher("/detection_info", ObjectPose, latch=True, queue_size=1)
        self.service = rospy.Service('detect', Empty, self.detect)

    def callback(self, msg):
        self.ros_cloud = msg
    
    def detect(self, req):
        cloud_objects = ros_to_pcl(self.ros_cloud)
        cluster_indices = euclidiean_cluster(cloud_objects)

        imagePub.capture_image()

        for index , pts_list in enumerate(cluster_indices):
            pcl_cluster = cloud_objects.extract(pts_list)
            pcl_cluster_arr = pcl_cluster.to_array()

            centroid = np.mean(pcl_cluster_arr , axis=0)[:3]
            centroid_point = Point()
            centroid_point.x = np.asscalar(centroid[0])
            centroid_point.y = np.asscalar(centroid[1])
            centroid_point.z = np.asscalar(centroid[2])

            max_val = np.max(pcl_cluster_arr, axis=0)[:2]
            min_val = np.min(pcl_cluster_arr, axis=0)[:2]
            
            x1, y1 = mapToImage(min_val[0], min_val[1], centroid[2])
            x2, y2 = mapToImage(max_val[0], max_val[1], centroid[2])

            #Classifier
            label = classifier.predict(pcl_cluster)
            rospy.loginfo("DETECTED " + label)

            imagePub.draw_rectangle_with_label([x1, y1, x2, y2], label)
            imagePub.publish_image()

            #Transform the centroid to base_link 
            centroid_point = transformer.transform_point(centroid_point, 'camera_rgb_frame2' , 'base_link')

            objPoseMsg = ObjectPose()
            objPoseMsg.name = label
            objPoseMsg.pose.header.frame_id = 'base_link'
            objPoseMsg.pose.header.stamp = rospy.Time.now()
            objPoseMsg.pose.pose.position = centroid_point
            self.detection_info_pub.publish(objPoseMsg)
            rospy.sleep(0.1)

            #width = np.asscalar(max_val[0] - min_val[0])
            #print("%d, %d, %d, %d %0.5f" % (x1, y1, x2, y2, width))

        return EmptyResponse()

def obj_detect_train(): #Function only used for training script
    #DEPRECATED - was used for tensorflow classification
    cloud_objects = ros_to_pcl(rospy.wait_for_message('/pcl_objects', PointCloud2))
    cluster_indices = euclidiean_cluster(cloud_objects)
    
    detected_obj = []

    for pts_list in cluster_indices:
        pcl_cluster_arr = cloud_objects.extract(pts_list).to_array()

        centroid = np.mean(pcl_cluster_arr , axis=0)[:3]
        centroid_point = Point()
        centroid_point.x = np.asscalar(centroid[0])
        centroid_point.y = np.asscalar(centroid[1])
        centroid_point.z = np.asscalar(centroid[2])
        c = centroid_point

        max_val = np.max(pcl_cluster_arr, axis=0)[:2]
        min_val = np.min(pcl_cluster_arr, axis=0)[:2]
    
        x1, y1 = mapToImage(min_val[0], min_val[1], centroid[2])
        x2, y2 = mapToImage(max_val[0], max_val[1], centroid[2])

        #width = np.asscalar(max_val[0] - min_val[0])
        # print("%d, %d, %d, %d %0.5f" % (x1, y1, x2, y2, width))
        detected_obj.append([x1, y1, x2, y2])

    return detected_obj

if __name__ == '__main__':
    rospy.init_node('obj_detection')
    transformer = Transformer()
    classifier = Classifier()
    objDetector = DetectObject()
    imagePub = ImagePub()
    rospy.loginfo("Started Object detection")
    rospy.spin()
