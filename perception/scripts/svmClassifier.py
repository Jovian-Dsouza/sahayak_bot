#!/usr/env/bin python

# Defines an SVM Classifier used for machine learning
# Trained SVM model https://drive.google.com/file/d/16kb5cKKvH0dWCTodaD-QVeIwyTxFxEze/view?usp=sharing
# Note: the trained SVM model needs to be in the models directory inside the perception package

import rospkg
import numpy as np
from feature_helper import extract_feature
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle

class Classifier:
    def __init__(self):
        #Load Model
        model_path = rospkg.RosPack().get_path('perception') + '/models/'
        model = pickle.load(open(model_path + 'SVMmodel.sav', 'rb'))
        self.clf = model['classifier']
        self.encoder = LabelEncoder()
        self.encoder.classes_ = model['classes']
        self.scaler = model['scaler']

    def predict(self, pcl_cluster):
        '''
        Take a pcl Cluster and predit its label
        '''
        feature = extract_feature(pcl_cluster)
        prediction = self.clf.predict(self.scaler.transform(feature.reshape(1,-1)))
        label = self.encoder.inverse_transform(prediction)[0]
        return label