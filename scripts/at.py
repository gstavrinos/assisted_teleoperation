#!/usr/bin/env python
import rospy
import tf
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

twist_publisher = None
obstacle_height = 0.136 #m
wheel_separation = 0.36 #m
found_sweet_spot = False

def approx(x, y):
    return abs(abs(x)-abs(y)) < 0.001

def imu_callback(msg):
    global pitch_threshold, twist_publisher, cnt, obstacle_height, wheel_separation, found_sweet_spot
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    #print "PITCH = " + str(pitch)
    sweet_spot = obstacle_height/wheel_separation;
    sweet_spot = math.sin(sweet_spot)
    sweet_spot = math.asin(sweet_spot)
    #print str(approx(sweet_spot, pitch))
    if approx(sweet_spot, pitch):
        found_sweet_spot = True
        print "SWEET SPOT = " + str(sweet_spot)
    if found_sweet_spot and abs(pitch) > 0.05:
        full_force = Twist()
        full_force.linear.x = 0.6
        twist_publisher.publish(full_force)
        print 'FULL FORCE!'
    elif found_sweet_spot:
        print 'NOW STOP!'
        twist_publisher.publish(Twist())
    #twist_publisher.publish(Twist())

def init():
    global pitch_threshold, twist_publisher
    rospy.init_node('assisted_teleoperation')
    imu_topic = rospy.get_param('~imu_topic','/imu/data')
    twist_topic = rospy.get_param('~twist_topic','/assisted_teleop/cmd_vel')
    rospy.Subscriber(imu_topic, Imu, imu_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=10);
    rospy.spin()



if __name__ == '__main__':
    init()
