#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib
#matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

background = None
plot = None
ax = None
X = None
Y = None
Z = None
linearx = 0
angularz = 0
prevx = -100
prevz = -100

def cmd_vel_callback(msg):
    global plot, background, ax, linearx, angularz
    linearx = msg.linear.x
    angularz = msg.angular.z

def image_callback(msg):
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.imshow("Video", image_np)
    cv2.waitKey(1)

def init():
    global plot, ax, X, Y, Z, background

    xx=np.linspace(-2,2,400)
    yy=np.linspace(-2,2,400)
    [X,Y]=np.meshgrid(xx,yy)
    Z=X*X+Y*Y

    plt.ion()
    plot = plt.figure()

    ax = plot.add_subplot(1, 1, 1)
    ax.set_aspect('equal')
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)

    background = plot.canvas.copy_from_bbox(ax.bbox)

    plt.show()
    plt.draw()
    plt.pause(0.0001)


if __name__ == '__main__':
    rospy.init_node('plot_imu_cmd_vel')
    rospy.Subscriber("/jaguar_velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, image_callback,  queue_size = 1)
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    init()
    p = ax.plot(angularz, linearx, 'o', color='k', markersize=50)[0]
    l = ax.plot([0, angularz], [0, linearx], '-o', color='k', linestyle='-', linewidth=2)
    c = plt.Circle((0, 0), 1, color='k', fill=False)
    while not rospy.is_shutdown():
        if prevx != linearx or prevz != angularz:
            p.set_data(angularz, linearx)
            l[0].set_data([0, angularz], [0, linearx])
            plot.canvas.restore_region(background)
            ax.add_artist(p)
            ax.add_artist(l[0])
            ax.add_artist(c)
            plot.canvas.blit(ax.bbox)
            prevx = linearx
            prevz = angularz

    cv2.destroyAllWindows()