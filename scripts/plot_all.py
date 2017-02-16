#!/usr/bin/env python
from __future__ import division

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import numpy as np
import os.path
import rosbag
import rospy
import sys
import tf

class terminal_colours:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

plot_figure = None
accx_plot = None
accy_plot = None
angz_plot = None
pose_plot = None
lx_plot = None # linear cmd_vel x
az_plot = None # angular cmd_vel z
first_message = True
starting_stamp = 0.0
latest_stamp = 0.0
_3d = False
colours = ['b', 'g', 'r', 'c', 'm', 'y']

def imu_callback(msg):
    global accx_plot, accy_plot, angz_plot, first_message, starting_stamp, latest_stamp, colours
    class_ = rospy.get_param("/class_", 0)

    if first_message:
        starting_stamp = (msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000)
        first_message = False

    colour = colours[class_]

    accx = msg.linear_acceleration.x
    accy = msg.linear_acceleration.y
    accx = msg.linear_acceleration.x

    velx = msg.angular_velocity.x
    vely = msg.angular_velocity.y
    velz = msg.angular_velocity.z

    t = (msg.header.stamp.secs + msg.header.stamp.nsecs / 1000000000) - starting_stamp
    latest_stamp = t

    accx_plot.scatter(t, accx, color=colour, marker=',', s=1)
    accy_plot.scatter(t, accy, color=colour, marker=',', s=1)
    angz_plot.scatter(t, velz, color=colour, marker=',', s=1)

    plt.draw()

def cmd_vel_callback(msg):
    global lx_plot, az_plot, starting_stamp, latest_stamp

    t = latest_stamp

    class_ = rospy.get_param("/class_", 0)

    colour = colours[class_]

    lx = msg.linear.x
    az = msg.angular.z

    lx_plot.scatter(t, lx, color=colour, marker=',', s=1)
    az_plot.scatter(t, az, color=colour, marker=',', s=1)
    plt.draw()

def pose_callback(msg):
    global pose_plot, _3d
    class_ = rospy.get_param("/class_", 0)

    colour = colours[class_]

    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    u = np.cos(yaw) * np.cos(pitch)
    v = np.sin(yaw) * np.cos(pitch)
    w = np.sin(pitch)

    if _3d:
        pose_plot.quiver(x, y, z, u, v, w, color=colour, length=0.08, normalize=True)
    else:
        pose_plot.quiver(x, y, u, v, color=colour)
    plt.draw()

def init():
    global plot_figure, accx_plot, accy_plot, angz_plot, lx_plot, az_plot, pose_plot
    plot_figure = plt.figure()
    accx_plot = plot_figure.add_subplot(3, 3, 1)
    accx_plot.set_title('Acceleration X')
    
    accy_plot = plot_figure.add_subplot(3, 3, 2, sharex= accx_plot, sharey=accx_plot)
    accy_plot.set_title('Acceleration Y')
    
    angz_plot = plot_figure.add_subplot(3, 3, 3, sharex= accx_plot)
    angz_plot.set_title('Angular Velocity Z')

    lx_plot = plot_figure.add_subplot(3, 3, 4, sharex= accx_plot)
    lx_plot.set_title('Linear cmd_vel X')
    
    az_plot = plot_figure.add_subplot(3, 3, 5, sharex= accx_plot, sharey=lx_plot)
    az_plot.set_title('Angular cmd_vel Z')

    pose_plot = plot_figure.add_subplot(3, 3, 6)
    pose_plot.set_title('Robot Position')

    adjustprops = dict(left=0.03, bottom=0.03, right=0.99, top=0.97, wspace=0.19, hspace=0.20)
    plot_figure.subplots_adjust(**adjustprops)


# Initialization
def online_init():
    global plot_figure, accx_plot, accy_plot, angz_plot, lx_plot, az_plot
    rospy.init_node('plot_imu_cmd_vel')
    init()
    rospy.Subscriber("/imu/data", Imu, imu_callback)
    rospy.Subscriber("/jaguar_velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)

    plt.show()

    while not rospy.is_shutdown():
        rospy.spin()

def offline_init():
    global plot_figure, accx_plot, accy_plot, angz_plot, lx_plot, az_plot, pose_plot, _3d
    init()
    arguments = sys.argv[1:] # exclude the script name
    cnt = -1
    for i in arguments:
        if str(i).endswith('.bag'):
            if os.path.isfile(i):
                cnt += 1
                bag = rosbag.Bag(i)
                accx = []
                accy = []
                angz = []
                lx = []
                az = []
                t1 = []
                t2 = []
                x = []
                y = []
                z = []
                u = []
                v = []
                w = []
                t3 = []
                print terminal_colours.OKBLUE + 'Processing ' + i + terminal_colours.ENDC
                for topic, msg, t in bag.read_messages(topics=['/imu/data', '/jaguar_velocity_controller/cmd_vel', '/amcl_pose']):
                    if topic == '/imu/data':
                        accx.append(msg.linear_acceleration.x)
                        accy.append(msg.linear_acceleration.y)
                        angz.append(msg.angular_velocity.z)
                        t1.append(t.secs + t.nsecs / 1000000000)

                    elif topic == '/amcl_pose':
                        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
                        x.append(msg.pose.pose.position.x)
                        y.append(msg.pose.pose.position.y)
                        z.append(msg.pose.pose.position.z)
                        u.append(np.cos(yaw) * np.cos(pitch))
                        v.append(np.sin(yaw) * np.cos(pitch))
                        w.append(np.sin(pitch))
                        t3.append(t.secs + t.nsecs / 1000000000)

                    else:
                        lx.append(msg.linear.x)
                        az.append(msg.angular.z)
                        t2.append(t.secs + t.nsecs / 1000000000)
                bag.close()
                for tt in range(1,len(t1)):
                    t1[tt] = t1[tt] - t1[0]
                t1[0] = 0
                for tt in range(1,len(t2)):
                    t2[tt] = t2[tt] - t2[0]
                t2[0] = 0
                for tt in range(1,len(t3)):
                    t3[tt] = t3[tt] - t3[0]
                t3[0] = 0
                colour = colours[cnt%len(colours)]
                accx_plot.plot(t1, accx, color=colour)
                accy_plot.plot(t1, accy, color=colour)
                angz_plot.plot(t1, angz, color=colour)
                lx_plot.plot(t2, lx, color=colour)
                az_plot.plot(t2, az, color=colour)
                if _3d:
                    pose_plot.quiver(x, y, z, u, v, w, color=colour, length=0.08, normalize=True)
                else:
                    pose_plot.quiver(x, y, u, v, color=colour)
                plt.draw()
            else:
                print terminal_colours.FAIL + i + ' does not exist!' + terminal_colours.ENDC
        else:
            print terminal_colours.FAIL + i + ' is not a rosbag!' + terminal_colours.ENDC
    plt.show()

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print terminal_colours.OKGREEN + 'Entering online mode. \nUse rosparam set class_:=0 / 1 / 2 to use different colours.' + terminal_colours.ENDC
        online_init()
    else:
        print terminal_colours.OKGREEN + 'Entering offline mode.' + terminal_colours.ENDC
        offline_init()