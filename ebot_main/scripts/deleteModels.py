#!/usr/bin/env python

import rospy
import random
import math
from math import pi
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


def delete_all_model():
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    for model in model_list:
        delete_model_prox(model)
    delete_model_prox('battery')
    rospy.sleep(1)

def delete_model(name):
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(name)

def wait_for_all_services():
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.loginfo("Connected to all services!")

model_list = ['robot_wheels', 'eYIFI', 'soap', 'coke_can', 'water_glass', 'adhesive', 'soap2' , 'glue']
realName_dict = {'robot_wheels' : 'robot_wheels' , 
                    'eYIFI' : 'eYFi_board', 
                    'soap' : 'FPGA_board' , 
                    'coke_can' : 'coke_can', 
                    'water_glass': 'water_glass', 
                    'adhesive' : 'adhesive', 
                    'soap2' : 'battery' , 
                    'glue' : 'glue'}

if __name__ == '__main__':
    rospy.init_node('reset_models')
    wait_for_all_services()
    delete_model('ebot')
    # delete_all_model()

