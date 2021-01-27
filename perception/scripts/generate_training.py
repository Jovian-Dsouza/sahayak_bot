#!/usr/bin/env python

"""
Helper script to generate training data for CNN classifier 
and SVM classifier
Captures random images of the models 
"""

from image_helper import image
from gazebo_helper import *
from objDetection import DetectObjectTrain
import random
import os
import rospkg

#For SVM
import pickle
from pcl_helper import *
from feature_helper import extract_feature

def random_pose(x_min, x_max , y_min, y_max, z=0.9):
    x = random.uniform(x_min, x_max)
    y = random.uniform(y_min, y_max)
    return create_pose_random_orient(x, y, z)


def scan(model):
    real_model_name = realName_dict[model]

    for i in range(n_scans):
        sample_was_good = False
        try_count = 0
        while not sample_was_good and try_count < 10:
            pose = random_pose(7.7, 8.1, 3.4, 3.6)
            spawn_model(model, pose)
            rospy.sleep(4)

            #capture 
            obj_list = detect.detect()

            pcl_cloud = detect.pcl_cloud
            
            # Check for invalids
            if  len(obj_list) != 1 :
                rospy.logerr('Invalid cloud detected : %d' % len(obj_list) )
                try_count += 1
                delete_model(model)
                rospy.sleep(0.5)
            else:
                sample_was_good = True
                #Save Image as real_model_name/n_scan.jpg
                rect = obj_list[0]
                sample_was_good = img_obj.crop_save_img(real_model_name+"/"+str(i)+".png", rect)
                rospy.sleep(0.2)
                img_obj.draw_rectangle(rect)
                img_obj.publish_image()

                if not sample_was_good :
                    delete_model(model)
                    rospy.sleep(0.5)

        # Extract histogram features
        feature = extract_feature(pcl_cloud)
        labeled_features.append([feature, real_model_name])
        rospy.loginfo("Scan Saved " + real_model_name + " : "+ str(i))

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
            os.mkdir( './' + model)  
        except OSError as error:  
            rospy.logerr(error)
    

if __name__ == '__main__':
    n_scans = 2
    model_list = ['robot_wheels', 'eYIFI', 'soap', 'coke_can', 'water_glass', 'adhesive', 'soap2' , 'glue']
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
    detect = DetectObjectTrain()

    delete_all_model(model_list)

    labeled_features = [] #For storing SVM training data

    for model in model_list:
        scan(model)

    pickle.dump(labeled_features, open('training_data.sav', 'wb'))

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
