#! /usr/bin/env python
import rospy
from transform_helper import Transformer
from perception.srv import *

def get_transform_point_cb(req):
    resp = GetTransformPointResponse()
    resp.point = transformer.transform_point(req.point, req.from_frame, req.to_frame)
    return resp

def get_transform_pose_cb(req):
    resp = GetTransformPoseResponse()
    resp.pose = transformer.transform_pose(req.pose, req.from_frame, req.to_frame)
    return resp

if __name__ == '__main__':
    rospy.init_node('transformer_service')
    transformer = Transformer()
    rospy.Service('get_transform_point', GetTransformPoint, get_transform_point_cb)
    rospy.Service('get_transform_pose', GetTransformPose, get_transform_pose_cb)
    rospy.loginfo('\033[94m Started Transformer service \033[0m')
    rospy.spin()