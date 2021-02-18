#!/usr/bin/env python

import rospy
import random
import math
from math import pi
import rospkg
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from gazebo_msgs.srv import GetPhysicsProperties
from gazebo_msgs.srv import SetPhysicsProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.srv import SetModelStateRequest
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest, SpawnModelResponse
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose
from sensor_msgs.msg import PointCloud2

deleteModel = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)

def delete_model(name):
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    delete_model_prox(name)

def create_pose(x, y, z ,qx ,qy ,qz ,qw):
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

def create_pose_angle(x, y, z , yaw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = quaternion_from_euler(0 ,0, yaw)
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
    # print("(%0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f, %0.8f)" % \
    #         (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
    _, _ , yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    print("'" + model_name+ "' : create_pose_angle(%0.5f, %0.5f, %0.5f, %0.5f)," %\
             (p.x, p.y, p.z, yaw))

def delete_all_model():
    delete_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    for model in model_list:
        delete_model_prox(model)
    delete_model_prox('battery')
    rospy.sleep(1)

def random_pose(x_min, x_max , y_min, y_max, z=0.9):
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return create_pose_random_orient(x, y, z)

def wait_for_all_services():
    rospy.wait_for_service('gazebo/get_model_state')
    rospy.wait_for_service('gazebo/set_model_state')
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    rospy.loginfo("Connected to all services!")

def spawn_eYIFI(y, angle=0):
    '''
    Angle in radians
    '''
    eYIFIPose = create_pose_angle(7.9712791, y, 1, pi+angle)
    spawn_model('eYIFI', eYIFIPose)
    
def spawn_FPGA(y, angle=0):
    '''
    Angle in radians
    '''
    FPGAPose = create_pose_angle(7.9712791, y, 0.9, pi/3 + angle)
    spawn_model('soap', FPGAPose)

def getAllStoreModels():
    printModelState('eYFi_board')
    printModelState('FPGA_board')
    printModelState('adhesive')
    printModelState('glue')
    printModelState('battery')
    printModelState('robot_wheels')

def resetModel(model, newPose):
    deleteModel(model)
    deleteModel(gazeboName[model])
    spawn_model(gazeboName[model], newPose)
    

model_list = ['robot_wheels', 'eYIFI', 'soap', 'coke_can', 'water_glass', 'adhesive', 'soap2' , 'glue']
realName_dict = {'robot_wheels' : 'robot_wheels' , 
                    'eYIFI' : 'eYFi_board', 
                    'soap' : 'FPGA_board' , 
                    'coke_can' : 'coke_can', 
                    'water_glass': 'water_glass', 
                    'adhesive' : 'adhesive', 
                    'soap2' : 'battery' , 
                    'glue' : 'glue'}

gazeboName = {'robot_wheels' : 'robot_wheels' , 
                    'eYFi_board':'eYIFI'  , 
                    'FPGA_board' : 'soap' , 
                    'coke_can' : 'coke_can', 
                    'water_glass': 'water_glass', 
                    'adhesive' : 'adhesive', 
                    'battery' :'soap2' , 
                    'glue' : 'glue'}

if __name__ == '__main__':
    rospy.init_node('reset_models')
    wait_for_all_services()
    # delete_all_model()

    cokePose = create_pose(7.9712791, 3.3939284, 0.8676281, -0.0126091, 0.0003598, 0.0000164, 0.9999204)
    gluePose = create_pose(7.84000000, 3.23928000, 0.86998147, 0.00000075, -0.00000197, 0.50251043, 0.86457115)
    batteryPose = create_pose(8.10856002, 3.23999991, 0.87299210, 0.00001689, 0.00000146, 0.00000001, 1.00000000)
    eYIFIPose = create_pose_angle(7.9712791, 3.2, 1, pi+ pi/4)
    adhesivePose = create_pose_angle(7.9712791, 3.2, 0.87299210, 0)
    FPGAPose = create_pose_angle(7.9712791, 3.2, 0.9, -pi/3)

    #store room
  
    storeTable = {'eYFi_board' : create_pose_angle(26.55000, -3.23000, 1.08999, -0.00011),
        'FPGA_board' : create_pose_angle(26.29000, -3.61300, 1.01500, -0.00000),
        'adhesive' : create_pose_angle(26.78000, -3.21000, 1.17000, -0.00000),
        'glue' : create_pose_angle(26.70000, -2.98400, 1.01450, 0.00000),
        'battery' : create_pose_angle(26.39500, -3.45000, 1.01750, -0.00000),
        'robot_wheels' : create_pose_angle(26.33000, -3.88012, 1.05310, 0.00002)}

    for model in storeTable.keys():
        resetModel(model, storeTable[model])
    