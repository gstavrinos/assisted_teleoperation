#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import matplotlib
matplotlib.use('Qt4Agg')
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

background = None
plot = None
ax = None
X = None
Y = None
Z = None

def cmd_vel_callback(msg):
    global plot, background, ax
    #plot.clear()
    #plot.contour(X, Y, Z, [1], color='k')
    #plot.scatter(msg.angular.z, msg.linear.x, s=1500, color='k')
    #plot.plot([0, msg.angular.z], [0, msg.linear.x], '-o', color='k', linestyle='-', linewidth=2)
    #plt.draw()
    #plt.pause(0.0001)

    #points = ax.contour(X, Y, Z, [1], color='k')

    #plot.canvas.restore_region(background)

    #ax.draw_artist(points)

    #plot.canvas.blit(ax.bbox)

    #print msg

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

    plot, ax = plt.subplots(1, 1)
    
    #ax.hold(True)
    
    background = plot.canvas.copy_from_bbox(ax.bbox)

    plt.draw()


    #plot.set_title('Command velocity')


if __name__ == '__main__':
    rospy.init_node('plot_imu_cmd_vel')
    init()
    rospy.Subscriber("/jaguar_velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, image_callback,  queue_size = 1)
    plt.show(False)
    cv2.namedWindow("Video", cv2.WINDOW_NORMAL)
    
    while not rospy.is_shutdown():
        rospy.spin()
    cv2.destroyAllWindows()