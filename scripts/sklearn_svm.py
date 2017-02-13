#!/usr/bin/env python
import os
import rospy
import rospkg
import numpy as np
import pandas as pd
from sklearn import svm
from geometry_msgs.msg import PoseWithCovarianceStamped

clf = None

def localization_callback(msg):
    global clf
    
    tmp = np.asmatrix([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    tmp = np.float32(tmp)
    y_val= clf.predict_proba(tmp)
    #print y_val
    if y_val[0][0] > y_val[0][1] and y_val[0][0] > y_val[0][2]:
        print str(y_val[0][0]*100) + '%: left'
    elif y_val[0][1] > y_val[0][0] and y_val[0][1] > y_val[0][2]:
        print str(y_val[0][1]*100) + '%: obstacle'
    elif y_val[0][2] > y_val[0][0] and y_val[0][2] > y_val[0][1]:
        print str(y_val[0][2]*100) + '%: parking'

# Initialization
def init():
    global clf
    rospy.init_node('sklearn_svm_test_node')

    rospack = rospkg.RosPack()
    training_data_path = rospack.get_path('assisted_teleoperation')+'/training_data/'
    first_time = True
    samples = []
    y_train = []

    for file in os.listdir(training_data_path):
        if file.endswith(".csv"):
            print file
            if first_time:
                first_time = False
                samples = np.genfromtxt(training_data_path+file, delimiter=',')
                y_train = np.empty(len(samples))
                if "left_turn" in file:
                    y_train.fill(0.)
                elif "obstacle" in file:
                    y_train.fill(1.)
                elif "parking" in file:
                    y_train.fill(2.)
            else:
                samples_ = np.genfromtxt(training_data_path+file, delimiter=',')
                y_train_ = np.empty(len(samples_))
                if "left_turn" in file:
                    y_train_.fill(0.)
                elif "obstacle" in file:
                    y_train_.fill(1.)
                elif "parking" in file:
                    y_train_.fill(2.)
                samples = np.append(samples, samples_, axis=0)
                y_train = np.append(y_train, y_train_, axis=0)

    samples = np.float32(samples)
    y_train = np.float32(y_train)
    
    if not first_time:
        clf = svm.SVC(probability=True)
        clf.fit(samples, y_train)

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, localization_callback)
        rospy.spin()
    else:
        print 'No training data found. Exiting.'

if __name__ == '__main__':
    init()