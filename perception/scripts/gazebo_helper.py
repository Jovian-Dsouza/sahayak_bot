#!/usr/bin/env python

import rospy
import random
import math
import rospkg
from tf.transformations import quaternion_from_euler

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def delete_model(model):
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(model)

def create_pose(x, y, z , qx, qy,qz,qw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    pose.orientation.x = qx
    pose.orientation.y = qy
    pose.orientation.z = qz
    pose.orientation.w = qw
    return pose

def create_pose_random_orient(x, y, z):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = quaternion_from_euler(0 , 0, random.uniform(0, 2*math.pi))
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

def spawn_model(model_name, pose):
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Spawning " + model_name)
    model_path = rospkg.RosPack().get_path('ebot_gazebo') \
                    + '/models/' + model_name + '/model.sdf'
    
    with open(model_path , 'r') as xml_file:
        model = xml_file.read()
    
    req = SpawnModelRequest()
    req.model_name = model_name
    req.model_xml = model
    req.initial_pose = pose

    spawn_srv.call(req)

def printModelState(model_name):
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox(model_name,'world')
    p = model_state.pose.position
    q = model_state.pose.orientation
    print("(%0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f)" % \
            (p.x, p.y, p.z, q.x, q.y, q.z, q.w))

def delete_all_model(model_list):
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    for model in model_list:
        delete_model_prox(model)
    rospy.sleep(1)

def wait_for_all_services():
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.wait_for_service('gazebo/delete_model')
    rospy.loginfo("Connected to all services!")

if __name__ == '__main__':
    rospy.init_node('capture_node')
    

    delete_all_model()

    cokePose = create_pose(7.9712791, 3.3939284, 0.8676281, -0.0126091, 0.0003598, 0.0000164, 0.9999204)
    gluePose = create_pose(7.84000000, 3.23928000, 0.86998147, 0.00000075, -0.00000197, 0.50251043, 0.86457115)
    batteryPose = create_pose(8.10856002, 3.23999991, 0.87299210, 0.00001689, 0.00000146, 0.00000001, 1.00000000)

    spawn_model('coke_can', cokePose)
    spawn_model('glue', gluePose)
    spawn_model('battery', batteryPose)
    
    # printModelState('coke_can')
    # printModelState('glue')
    # printModelState('battery')