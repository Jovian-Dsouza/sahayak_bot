#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import matplotlib.pyplot as plt
from tensorflow import keras
from std_srvs.srv import Empty, EmptyResponse
import rospkg

class Classifier:
    def __init__(self):
        #Load Model
        model_path = rospkg.RosPack().get_path('perception') + '/models/'
        self.model = keras.models.load_model(model_path + 'eyaantra_8_label_classifier.h5')
        self.HEIGHT = 80
        self.WIDTH = 50
        self.labels = {
                0 : 'adhesive',  
                1 : 'coke_can',    
                2 : 'FPGA_board',  
                3 : 'robot_wheels',
                4 : 'battery',   
                5 : 'eYFi_board',  
                6 : 'glue',	  
                7 : 'water_glass',
               -1 : 'unknown'
        }

    def predictlabel(self, cv_image):
        '''
        returns {
            0 - adhesive  
            1 - coke_can    
            2 - FPGA_board  
            3 - robot_wheels
            4 - battery   
            5 - eYFi_board  
            6 - glue	  
            7 - water_glass
        }
        '''

        cv_image = cv2.resize(cv_image, (self.WIDTH, self.HEIGHT))
        cv_image = cv_image / 255.0

        cv_image = np.array(cv_image)
        to_pred = np.array([cv_image])

        print(to_pred.shape)

        pred = self.model.predict(to_pred)
        print(pred)

        mx_idx = np.argmax(pred, axis=1)[0]
        label_idx = mx_idx if pred[0][mx_idx] > 0.9 else -1
        return self.labels[label_idx]

def test(req):
    # testing
    cv_image = cv2.imread(rospkg.RosPack().get_path('perception') + '/data/' +'task5_dataset/robot_wheels/0.png')
    print(ob.predictlabel(cv_image))
    return EmptyResponse()

if __name__ == "__main__":
    ob = Classifier()
    rospy.init_node("tensorflow_classifier")
    rospy.Service('/ebot/classifier', Empty, test)
    rospy.loginfo(">> Tensorflow Service Started : /ebot/classifier")
    rospy.spin()