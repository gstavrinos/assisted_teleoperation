#!/usr/bin/env python
import tf
import math
import time
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

twist_publisher = None
obstacle_height = 0.11 #0.136 #m
wheel_separation = 0.36 #m
found_sweet_spot = False
starting_time = 0
teleop_command_received = False


# Approximate two values with l precision
def approx(x, y, l):
    return abs(abs(x)-abs(y)) < l


# Teleoperation callback
def teleop_callback(msg):
    global teleop_command_received
    if msg.linear.x > 0:
        teleop_command_received = True


# IMU callback
def imu_callback(msg):
    global teleop_command_received
    if teleop_command_received:
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        assisted_teleop(pitch)
        teleop_command_received = False

# The main rationale of the program
def assisted_teleop(pitch):
    global twist_publisher, obstacle_height, wheel_separation, found_sweet_spot, starting_time
    #print "PITCH = " + str(pitch)
    sweet_spot = obstacle_height/wheel_separation;
    sweet_spot = math.sin(sweet_spot)
    sweet_spot = math.asin(sweet_spot)
    #print str(approx(sweet_spot, pitch))
    if approx(sweet_spot, pitch, 0.01): #0.001):
        found_sweet_spot = True
        print "SWEET SPOT = " + str(sweet_spot)
        starting_time = time.time()
    if found_sweet_spot and not approx(pitch, 0, 0.1):
        now = time.time()
        if now - starting_time <= 2:
            full_force = Twist()
            full_force.linear.x = 0.6
            twist_publisher.publish(full_force)
            print 'FULL FORCE!'
        else: # We are most probably past the sweet spot, so let's get slowly back
            back = Twist()
            back.linear.x = -0.15
            twist_publisher.publish(back)
    elif found_sweet_spot:
        print 'NOW STOP!'
        found_sweet_spot = False
        twist_publisher.publish(Twist())


# Initialization
def init():
    global twist_publisher, wheel_separation
    rospy.init_node('assisted_teleoperation')
    imu_topic = rospy.get_param('~imu_topic','/imu/data')
    twist_topic = rospy.get_param('~twist_topic','/assisted_teleop/cmd_vel')
    wheel_separation = rospy.get_param('~wheel_separation', 0.36)
    teleop_cmd_vel_topic = rospy.get_param('~teleop_cmd_vel_topic', '/joy_teleop/cmd_vel')
    rospy.Subscriber(imu_topic, Imu, imu_callback)
    rospy.Subscriber(teleop_cmd_vel_topic, Twist, teleop_callback)
    twist_publisher = rospy.Publisher(twist_topic, Twist, queue_size=10);
    rospy.spin()



if __name__ == '__main__':
    init()
