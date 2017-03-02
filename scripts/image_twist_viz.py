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
linearx2 = []
angularz2 = []
prevx = -100
prevz = -100
starting_stamp = 0.0
latest_stamp = 0.0

def cmd_vel_callback(msg):
    global plot, background, ax, linearx, angularz, linearx2, angularz2, latest_stamp
    linearx = msg.linear.x
    angularz = msg.angular.z
    found = False
    for i in range(len(linearx2)):
        if linearx2[i] == linearx and angularz2 == angularz:
            found = True
            break
    if not found:
        linearx2.append(msg.linear.x)
        angularz2.append(msg.angular.z)

def image_callback(msg):
    global starting_stamp, latest_stamp, linearx2, angularz2
    if (starting_stamp == 0):
        starting_stamp = (msg.header.stamp.secs + msg.header.stamp.nsecs / 1E9)
    curr = ((msg.header.stamp.secs + msg.header.stamp.nsecs / 1E9) - starting_stamp)
    if curr < latest_stamp:
        linearx2 = []
        angularz2 = []
    np_arr = np.fromstring(msg.data, np.uint8)
    latest_stamp = curr
    current_time = str(latest_stamp)
    current_time = current_time[:current_time.find('.')+2]
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    cv2.putText(image_np, current_time, (50, 50), cv2.FONT_HERSHEY_DUPLEX, 2, (0, 0, 255))
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
    #plt.pause(0.0001)


if __name__ == '__main__':
    rospy.init_node('plot_imu_cmd_vel')
    rospy.Subscriber("/jaguar_velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/axis/image_raw/compressed", CompressedImage, image_callback,  queue_size = 1)
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    init()
    p = ax.plot(angularz, linearx, 'o', color='k', markersize=50)[0]
    p2 = ax.plot(angularz, linearx, color='r', markersize=3)[0]
    #l = ax.plot([0, angularz], [0, linearx], '-o', color='k', linestyle='-', linewidth=2)
    c = plt.Circle((0, 0), 1, color='k', fill=False)
    ax.add_artist(c)
    while not rospy.is_shutdown():
        if True: #prevx != linearx or prevz != angularz:
            p.set_data(angularz, linearx)
            p2.set_data(angularz2, linearx2)
            #l[0].set_data([0, angularz], [0, linearx])
            #plot.canvas.restore_region(background)
            #ax.add_artist(p)
            #ax.add_artist(l[0])
            plot.canvas.blit(ax.bbox)
            prevx = linearx
            prevz = angularz
            plot.canvas.flush_events()

    cv2.destroyAllWindows()