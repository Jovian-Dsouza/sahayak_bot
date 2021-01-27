#!/usr/bin/env python
import numpy as np
import pickle
import rospy
import random
import math
import rospkg
from tf.transformations import quaternion_from_euler

from pcl_helper import *
from feature_helper import extract_feature
from objDetection import DetectObjectTrain

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2


def delete_model():
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox('training_model')

def initial_setup():

    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/get_physics_properties')
    rospy.wait_for_service('gazebo/set_physics_properties')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.loginfo("Connected to all services!")

    # get_physics_properties_prox = rospy.ServiceProxy('gazebo/get_physics_properties', GetPhysicsProperties)
    # physics_properties = get_physics_properties_prox()

    # physics_properties.gravity.z = 0

    # set_physics_properties_prox = rospy.ServiceProxy('gazebo/set_physics_properties', SetPhysicsProperties)
    # set_physics_properties_prox(physics_properties.time_step,
    #                             physics_properties.max_update_rate,
    #                             physics_properties.gravity,
    #                             physics_properties.ode_config)


    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    # delete_model_prox('ground_plane')
    # delete_model_prox('office')
    delete_model_prox('dropbox')
    # delete_model_prox('table')
    delete_model_prox('coke_can')
    delete_model_prox('glue')
    delete_model_prox('battery')
    rospy.sleep(1)



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

def random_pose(x_min, x_max , y_min, y_max, z=0.9):
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return create_pose_random_orient(x, y, z)

def spawn_model(model_name):
    spawn_srv = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    rospy.loginfo("Spawning " + model_name)
    model_path = rospkg.RosPack().get_path('ebot_gazebo') \
                    + '/models/' + model_name + '/model.sdf'
    
    with open(model_path , 'r') as xml_file:
        model = xml_file.read()
    
    req = SpawnModelRequest()
    req.model_name = 'training_model'
    req.model_xml = model
    req.initial_pose = create_pose_random_orient(7.971279, 3.2,  1.0)

    spawn_srv.call(req)
    rospy.sleep(1)

def random_orient():
    '''
    Sets random orientation
    '''
    get_model_state_prox = rospy.ServiceProxy('gazebo/get_model_state',GetModelState)
    model_state = get_model_state_prox('training_model','world')

    set_model_state_prox = rospy.ServiceProxy('gazebo/set_model_state', SetModelState)

    # model_state.pose = random_pose(7.7, 8.1, 3.2, 4)
    model_state.pose = create_pose_random_orient(7.971279, 3.2,  1.0)

    sms_req = SetModelStateRequest()
    sms_req.model_state.pose = model_state.pose
    sms_req.model_state.twist = model_state.twist
    sms_req.model_state.model_name = 'training_model'
    sms_req.model_state.reference_frame = 'world'
    set_model_state_prox(sms_req)

def capture_sample():
    """
    captures a point cloud of the model 
    """
    return rospy.wait_for_message('/camera2/depth/points2', PointCloud2)


if __name__ == '__main__':
    rospy.init_node('capture_node')

    n_scans_per_object = 20
    models = ['coke_can', 'adhesive', 'robot_wheels', 'eYIFI', 'soap', 'water_glass', 'soap2' , 'glue']
    realName_dict = {'robot_wheels' : 'robot_wheels' , 
                     'eYIFI' : 'eYFi_board', 
                     'soap' : 'FPGA_board' ,  
                     'coke_can' : 'coke_can', 
                     'water_glass': 'water_glass', 
                     'adhesive' : 'adhesive', 
                     'soap2' : 'battery' , 
                     'glue' : 'glue'}

    # Disable gravity and delete the ground plane
    initial_setup()
    detect = DetectObjectTrain()
    labeled_features = []

    
    for model_name in models:
        spawn_model(model_name)
        rospy.sleep(1)
        for i in range(n_scans_per_object):
            # make five attempts to get a valid a point cloud then give up
            sample_was_good = False
            try_count = 0
            while not sample_was_good and try_count < 5:
                random_orient()
                rospy.sleep(5)
                ros_cloud = detect.ros_cloud #capture_sample()
                pcl_cloud = ros_to_pcl(ros_cloud)
                pcl_cloud_arr = pcl_cloud.to_array()

                # Check for invalid clouds.
                if  pcl_cloud_arr.shape[0] == 0:
                    print('Invalid cloud detected')
                    try_count += 1
                else:
                    sample_was_good = True

            # Extract histogram features
            feature = extract_feature(pcl_cloud)
            real_model_name = realName_dict[model_name]
            labeled_features.append([feature, real_model_name])

        delete_model()
        rospy.sleep(1)


    pickle.dump(labeled_features, open('svm/training_data.sav', 'wb'))

