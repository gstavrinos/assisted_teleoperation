#!/usr/bin/env python
import sys
import cv2
import rospy
import Image
import numpy as np
import pyqtgraph as pg
from PyQt4 import QtGui
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage

imageview = None
plot = None
app = None
w = None

def cmd_vel_callback(msg):
    global plot
    #print msg

def image_callback(msg):
    global imageview
    np_arr = np.fromstring(msg.data, np.uint8)
    image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
    #im = Image.fromarray(image_np)
    imageview.setImage(image_np.transpose([2,1,0]))
    #im.show()
    #print msg

def init_GUI():
    global imageview, plot, w, app
    app = QtGui.QApplication(sys.argv)
    w = QtGui.QWidget()
    layout = QtGui.QGridLayout()
    imageview = pg.ImageView()
    imageview.ui.histogram.hide()
    imageview.ui.roiBtn.hide()
    imageview.ui.menuBtn.hide()
    imageview.ui.roiPlot.hideAxis('bottom')
    plot = pg.PlotWidget()
    w.resize(250, 150)
    w.move(300, 300)
    w.setWindowTitle('Joy Viz')
    layout.addWidget(imageview)
    layout.addWidget(plot)
    w.setLayout(layout)
    w.show()


if __name__ == '__main__':
    rospy.init_node('plot_imu_cmd_vel')
    init_GUI()
    rospy.Subscriber("/jaguar_velocity_controller/cmd_vel", Twist, cmd_vel_callback)
    rospy.Subscriber("/usb_cam/image_raw/compressed", CompressedImage, image_callback,  queue_size = 1)

    sys.exit(app.exec_())
    
    #while not rospy.is_shutdown():
    #    rospy.spin()