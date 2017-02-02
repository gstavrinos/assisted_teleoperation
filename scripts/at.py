#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

twist_publisher = None

def imu_callback(msg):
    global pitch_threshold, twist_publisher, cnt
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    print "PITCH = " + str(pitch)
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
