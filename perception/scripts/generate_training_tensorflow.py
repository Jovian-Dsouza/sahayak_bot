#!/usr/bin/env python

"""
Helper script to generate training data for CNN classifier
Captures random images of the models 
"""

from image_helper import image
from gazebo_helper import *
from objDetection import obj_detect_train
import random
import os
import rospkg

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


def scan(model):
    for i in range(n_scans):
        sample_was_good = False
        try_count = 0
        while not sample_was_good and try_count < 5:
            pose = create_pose_random_orient(7.971279, 3.2,  1.0)
            spawn_model(model, pose)
            rospy.sleep(4)

            #capture
            obj_list = obj_detect_train()
            
            # Check for invalids
            if  len(obj_list) != 1 :
                rospy.logerr('Invalid cloud detected : %d' % len(obj_list) )
                try_count += 1
                delete_model(model)
                rospy.sleep(0.5)
            else:
                sample_was_good = True
                #Save Image as model_name/n_scan.jpg
                rect = obj_list[0]
                sample_was_good = img_obj.crop_save_img(realName_dict[model]+"/"+str(i)+".png", rect)
                rospy.sleep(0.2)
                img_obj.draw_rectangle(rect)
                img_obj.publish_image()

                delete_model(model)
                rospy.sleep(0.5)

def init_dir(folder_name, model_list):
    parent = rospkg.RosPack().get_path('perception')
    path = parent + "/data/" + folder_name
    try:
        os.rmdir(path)
    except OSError as error: 
        rospy.logerr(error) 
    
    try:  
        os.mkdir(path)  
    except OSError as error:  
        rospy.logerr(error) 

    try:  
        os.chdir(path)  
    except OSError as error:  
        rospy.logerr(error) 
    
    for model in model_list:
        try:  
            os.mkdir( './' + realName_dict[model])  
        except OSError as error:  
            rospy.logerr(error)
    

if __name__ == '__main__':
    n_scans = 30
    model_list = ['adhesive','coke_can', 'robot_wheels', 'eYIFI', 'soap', 'water_glass', 'soap2' , 'glue']
    realName_dict = {'robot_wheels' : 'robot_wheels' , 
                     'eYIFI' : 'eYFi_board', 
                     'soap' : 'FPGA_board' ,  
                     'coke_can' : 'coke_can', 
                     'water_glass': 'water_glass', 
                     'adhesive' : 'adhesive', 
                     'soap2' : 'battery' , 
                     'glue' : 'glue'}

    init_dir('task5_dataset', model_list)
    rospy.init_node('generate_training', anonymous=True)
    wait_for_all_services()

    img_obj = image()

    delete_all_model(model_list)

    for model in model_list:
        scan(model)

    # model = 'battery' 
    # scan(model)

    # while not rospy.is_shutdown():
    #     pose = random_pose(7.7, 8.1, 3.2, 4)
    #     spawn_model(model, pose)
    #     rospy.sleep(4)

    #     #capture
    #     obj_list = obj_detect_train()
    #     rect = obj_list[0]
    #     #img_obj.capture_image()
    #     img_obj.draw_rectangle(rect)
    #     img_obj.publish_image()

    #     #delete
    #     delete_model(model)
    #     rospy.sleep(0.5)
