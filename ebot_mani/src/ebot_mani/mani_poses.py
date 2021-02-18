#! /usr/bin/env python

from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from math import pi

def create_pose_quaternion(x, y, z ,qx ,qy ,qz ,qw):
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

def create_pose_euler(x, y, z, roll, pitch, yaw):
    '''
    returns a Pose() object from the given x, y, z, qx, qy , qz, qw values
    '''
    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    q = quaternion_from_euler(roll, pitch , yaw)

    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose

joint_angles_dict = {
    "homePose" : [0.0, -0.5901, 0.0, -1.5708, -0.5235, 1.5708 ],
    "homePose1" : [0.0, -0.5901, -0.2, -1.3314, -0.5235, 1.5708 ],
    "navPoseOld" : [0.0, -0.7290, 0.2256, 0.5207, 1.5700, 0.0],
    "navPose" : [0.0, 0.0, 0.6769, 5.0334, 1.5700, 0.0],
    "frontHomePose" : [0.0, -0.3124, -0.83, 1.0067, 1.5700, 0.0 ],
    "trainingPose" : [0.0, -0.5901, -0.2, -1.3314, -0.5235, 1.5708 ],
    "researchDropboxJ" : [0, -0.66, -0.711, -2.395, -1.57, 3.14],
    "seeObjectJ" : [-5.149964, -1.328160, -0.471244, 5.101164, -2.598675, -2.533228],
    "graspVerticalJ" : [-0.405564, -0.287673, -0.682579, -1.123483, 4.712032, 0.117346]
}

poses_dict = { #w.r.t base_link -0.65
    "pantryTable1" : create_pose_euler(0.146625 , -0.17, 0.3, -pi + 5 *(pi/180), 0.0, 1.571713),
    "meetingDropbox" : create_pose_euler(0.7067 , 0.08208, 0.5513134, 3.041132, 0.0, 1.571657),
    "conferenceDropbox" : create_pose_euler(0.45 , 0.3, 0.613080, 3.040883, 0.001715, 1.571479),
    "homePoseNew" : create_pose_euler(-0.6 , -0.0, 0.7, -pi + 15 *(pi/180), 0.0, -pi-pi/2),

    "storeTable" : create_pose_euler(.146583 , 0.109158, 0.65, -pi + 19 *(pi/180), -0.0, 1.5716425),

    "frontHomePoseNew" : create_pose_euler(-0.05 , -0.2, 0.54, -pi + 32 *(pi/180), 0.0 , 1.571546),
    "storeHomePose" : create_pose_euler(-0.2 , 0.109151, 0.64, -pi + 5 *(pi/180), 0.0, 1.571451),
    "seeObjectPose" : create_pose_euler(-0.199977 , -0.17, 0.640006, -pi + 19 *(pi/180), 0.000612, 1.571682),
    "meetingTable" : create_pose_euler(0.060330 , 0.25, 0.557052, -2.600618, -0.000331, 1.571597),
    "researchDropbox" : create_pose_euler(0.5 , 0.109149, 0.813080, 3.040883, 0.001715, 1.571479)
}

odom_poses_dict ={ #w.r.t odom 
    "pantryTable1Odom" : create_pose_euler(11.290917 , -1.100473, 0.955937, -3.053954, -0.000183, -1.468141),
    "seeObjectPoseOdom" : create_pose_euler(25.689037 , -3.334134, 1.295877, -2.809943, 0.000944, 0.967247),
    "homePoseNewOdom" : create_pose_euler(25.415135 , -2.660356, 1.255860, -2.880520, -0.000369, 0.964223),
    "meetingDropboxOdom" : create_pose_quaternion(7.198955 , 3.043471, 1.207085, 0.021290, 0.998542, 0.049611, 0.00053),
    "conferenceDropboxOdom" : create_pose_quaternion(5.580729 , -0.832483, 1.268945, 0.998411, -0.025790, -0.002579, 0.050030),
    "storeHomePoseOdom" : create_pose_euler(25.171662 , -2.576347, 1.255974, -3.054999, -0.001029, 0.960576),
    "navPoseOdom" : create_pose_euler(0.038781 , 0.096871, 0.972988, -2.600636, -0.000253, 1.594896),
    "meetingTableOdom" : create_pose_quaternion(8.039303 , 2.696929, 1.213057, -0.012226, 0.963572, -0.267144, 0.003609),
    "researchDropboxOdom" : create_pose_quaternion(11.290601 , 9.488086, 1.468924, 0.715182, 0.697136, 0.034512, 0.036406),
    "storeTableOdom" : create_pose_euler(25.760034 , -3.263913, 1.305903, -2.810132, -0.000961, 0.950947),
    "fpgaPoseOdom" : create_pose_euler(25.648084 , -3.261405, 1.295843, -2.809304, 0.000646, 0.962793)
}