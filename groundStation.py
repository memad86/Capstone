#!/usr/bin/env python3

import sys
import rospy
import time
from PyQt5.QtWidgets import *
from std_msgs.msg import Float64, Bool, Float64MultiArray
from geometry_msgs.msg import Point
from mavros_msgs.msg import State

class MyWindow(QWidget):
    def __init__(self, id):
        super().__init__()
        self.uav_ID = id
        self.state  = State()
        # Initialize ROS node
        rospy.init_node("uav_" + str(self.uav_ID) + "_GroundStation_node", anonymous=False)

        #Publishers
        self.takeoff_pub        = rospy.Publisher("/height", Point, queue_size=10)
        self.lat_long_pub       = rospy.Publisher("/target_position", Float64MultiArray, queue_size=10)
        self.speed_pub          = rospy.Publisher("/prefered_velocity", Float64, queue_size=10)
        self.takeOff_pub        = rospy.Publisher("/takeOff", Bool, queue_size=10)
        self.land_pub           = rospy.Publisher("/landing", Bool, queue_size=10)
        self.offboard_pub       = rospy.Publisher("/offboard", Bool, queue_size=10)
        self.velocity_mode_pub  = rospy.Publisher("/velocity_mode", Bool, queue_size=10)

        #Subscribers
        self.state_sub          = rospy.Subscriber("/mavros/state", State, self.stateCallback)

        # Set window size
        self.setGeometry(200, 200, 400, 250)

        # Create buttons
        self.takeoff_button = QPushButton('TakeOff', self)
        self.takeoff_button.clicked.connect(self.takeoff)

        self.motion_button = QPushButton('Motion to Target', self)
        self.motion_button.clicked.connect(self.motion)

        self.speed_button = QPushButton('Set Speed', self)
        self.speed_button.clicked.connect(self.set_speed)

        self.land_button = QPushButton('Land', self)
        self.land_button.clicked.connect(self.land)

        # Create text boxes and labels
        self.latitude_label = QLabel('Start Lat:', self)
        self.latitude_textbox = QLineEdit(self)

        self.longitude_label = QLabel('Start Long:', self)
        self.longitude_textbox = QLineEdit(self)

        self.end_latitude_label = QLabel('End Lat:', self)
        self.end_latitude_textbox = QLineEdit(self)

        self.end_longitude_label = QLabel('End Long:', self)
        self.end_longitude_textbox = QLineEdit(self)

        self.rowCount_label = QLabel('No of Rows:', self)
        self.rowCount_textbox = QLineEdit(self)

        self.takeoff_height_label = QLabel('Takeoff Height:', self)
        self.takeoff_height_textbox = QLineEdit(self)

        self.speed_label = QLabel('Speed (m/s):', self)
        self.speed_textbox = QLineEdit(self)

        # Set layout
        grid_layout = QGridLayout()
        grid_layout.addWidget(self.takeoff_button, 0, 0, 2, 1)
        grid_layout.addWidget(self.motion_button, 3, 0, 2, 1)
        grid_layout.addWidget(self.speed_button, 8, 0, 2, 1)
        grid_layout.addWidget(self.land_button, 10, 0, 2, 1)

        grid_layout.addWidget(self.takeoff_height_label, 0, 1)
        grid_layout.addWidget(self.takeoff_height_textbox, 0, 2)

        grid_layout.addWidget(self.latitude_label, 2, 1)
        grid_layout.addWidget(self.latitude_textbox, 2, 2)

        grid_layout.addWidget(self.longitude_label, 3, 1)
        grid_layout.addWidget(self.longitude_textbox, 3, 2)

        grid_layout.addWidget(self.end_latitude_label, 4, 1)
        grid_layout.addWidget(self.end_latitude_textbox, 4, 2)

        grid_layout.addWidget(self.end_longitude_label, 5, 1)
        grid_layout.addWidget(self.end_longitude_textbox, 5, 2)

        grid_layout.addWidget(self.rowCount_label, 6, 1)
        grid_layout.addWidget(self.rowCount_textbox, 6, 2)

        grid_layout.addWidget(self.speed_label, 8, 1)
        grid_layout.addWidget(self.speed_textbox, 8, 2)

        # grid_layout.setColumnStretch(0, 1)
        # grid_layout.setColumnStretch(1, 1)
        # grid_layout.setColumnStretch(2, 2)

        self.setLayout(grid_layout)


    def stateCallback(self, data):
        self.state = data

    def takeoff(self):
        takeoff_height = Point()
        takeoff_height.z = float(self.takeoff_height_textbox.text())
        self.takeOff_pub.publish(True)
        self.land_pub.publish(False)
        if self.state.armed:
            self.offboard_pub.publish(True)
            self.velocity_mode_pub.publish(False)
            self.takeoff_pub.publish(takeoff_height)
            print('Taking off to height:', takeoff_height)

    def motion(self):
        targetPosition = Float64MultiArray()
        startLat = float(self.latitude_textbox.text())
        startLong = float(self.longitude_textbox.text())
        endLat = float(self.end_latitude_textbox.text())
        endLong = float(self.end_longitude_textbox.text())

        rowCount = float(self.rowCount_textbox.text())

        targetPosition.data = [startLat, startLong, endLat, endLong, rowCount]

        self.takeOff_pub.publish(True)
        self.land_pub.publish(False)
        self.offboard_pub.publish(True)
        self.velocity_mode_pub.publish(True)
        self.lat_long_pub.publish(targetPosition)
        # print('Moving to latitude:', targetPosition.x, 'and longitude:', targetPosition.y)

    def set_speed(self):
        speed = float(self.speed_textbox.text())
        self.speed_pub.publish(speed)
        print('Setting speed to:', speed)

    def land(self):
        self.takeOff_pub.publish(True)
        self.land_pub.publish(True)
        print('Setting Land True')

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MyWindow(0)
    window.show()
    sys.exit(app.exec_())