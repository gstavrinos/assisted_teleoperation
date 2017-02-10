#!/usr/bin/env python
import numpy as np
import pandas as pd
import cv2
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped

clf = None

class StatModel(object):
    def load(self, fn):
        self.model.load(fn)
    def save(self, fn):
        self.model.save(fn)

class SVM(StatModel):
    def __init__(self):
        self.model = cv2.SVM()

    def train(self, samples, responses):
        params = dict( kernel_type = cv2.SVM_LINEAR, 
                       svm_type = cv2.SVM_C_SVC,
                       C = 1 )
        self.model.train(samples, responses, params = params)

    def predict(self, samples):
        return np.float32( [self.model.predict(s) for s in samples])


def localization_callback(msg):
    global clf
    #samples_ = np.genfromtxt('/home/gstavrinos/ros_packages/other/src/assisted_teleoperation/scripts/cfou_parking_2-amcl_pose.csv', delimiter=',')
    #samples_ = np.float32(samples_)
    
    #tmp = np.empty(7)
    tmp = np.asmatrix([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z,msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w])
    tmp = np.float32(tmp)
    y_val = clf.predict(tmp)
    if y_val < 1:
        print 'left'
    elif y_val < 2:
        print 'obstacle'
    elif y_val < 3:
        print 'parking'
    #print y_val


# Initialization
def init():
    global clf
    rospy.init_node('svm_test_node')
    
    samples = np.genfromtxt('/home/gstavrinos/ros_packages/other/src/assisted_teleoperation/scripts/cfou_left_turn_3-amcl_pose.csv', delimiter=',')
    y_train = np.empty(len(samples))
    y_train.fill(0.)

    samples_ = np.genfromtxt('/home/gstavrinos/ros_packages/other/src/assisted_teleoperation/scripts/cfou_obstacle_1-amcl_pose.csv', delimiter=',')
    y_train_ = np.empty(len(samples_))
    y_train_.fill(1.)
    samples = np.append(samples, samples_, axis=0)
    y_train = np.append(y_train, y_train_, axis=0)

    samples_ = np.genfromtxt('/home/gstavrinos/ros_packages/other/src/assisted_teleoperation/scripts/cfou_parking_1-amcl_pose.csv', delimiter=',')
    y_train_ = np.empty(len(samples_))
    y_train_.fill(2.)
    samples = np.append(samples, samples_, axis=0)
    y_train = np.append(y_train, y_train_, axis=0)

    samples = np.float32(samples)
    y_train = np.float32(y_train)

    clf = SVM()
    clf.train(samples, y_train)

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, localization_callback)
    '''
    imu_topic = rospy.get_param('~imu_topic','/imu/data')
    twist_topic = rospy.get_param('~twist_topic','/assisted_teleop/cmd_vel')
    wheel_separation = rospy.get_param('~wheel_separation', 0.36)
    teleop_cmd_vel_topic = rospy.get_param('~teleop_cmd_vel_topic', '/joy_teleop/cmd_vel')
    rospy.Subscriber(imu_topic, Imu, imu_callback)
    rospy.Subscriber(teleop_cmd_vel_topic, Twist, teleop_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=10);
    '''
    rospy.spin()
    
    print y_val

if __name__ == '__main__':
    init()