#!/usr/bin/env python
import tf
import rospy
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import PoseWithCovarianceStamped

plot_figure = None
plot = None

def pose_callback(msg):
    global plot
    (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    z = msg.pose.pose.position.z

    u = np.cos(yaw) * np.cos(pitch)
    v = np.sin(yaw) * np.cos(pitch)
    w = np.sin(pitch)

    plot.quiver(x, y, z, u, v, w, length=0.08, color='red', normalize=True)
    plt.draw()

# Initialization
def init():
    global plot_figure, plot
    rospy.init_node('plot_pose')

    plot_figure = plt.figure()
    plot = plot_figure.add_subplot(1, 1, 1, projection='3d')
    plot.set_title('Pose Plot')

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, pose_callback)
    plt.show()
    while not rospy.is_shutdown():
        rospy.spin()

if __name__ == '__main__':
    init()