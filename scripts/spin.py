#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist#, Vector3
from sensor_msgs.msg import Image#, LaserScan
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge
import cv2
from tf.transformations import euler_from_quaternion
# import numpy as np
import math
import os



class StreetViewDriver(object):

    def __init__(self, image_topic, photoNumber, base_path):
        rospy.init_node('streetviewdriver')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self.odom_signal)
        rospy.Subscriber(image_topic, Image, self.camera_signal)
        self.bridge = CvBridge()
        # cv2.namedWindow('video_window')

        
        # initialization
        self.base_path = base_path
        self.odom = None
        self.cv_image = None
        self.x = None
        self.y = None
        self.yaw = None
        self.twist = Twist()
        self.angularVelocity = 0.25
        self.photoNumber = photoNumber
        self.diff = 2*math.pi/self.photoNumber
        self.theta_start = None

        while self.x is None or self.yaw is None:
            print "waiting for pos and degrees"
            continue
        self.lastDegrees = self.yaw
        self.create_pose_folder()

    @staticmethod
    def convert_pose_to_xy_and_theta(pose):
        """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
        orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        angles = euler_from_quaternion(orientation_tuple)
        return pose.position.x, pose.position.y, angles[2]

    def create_pose_folder(self):
        pose_folder = "{x},{y}".format(x=round(self.x, 0), y=round(self.y, 0))
        path = os.path.join(self.base_path, 'raw', pose_folder)
        self.save_path = path
        if not os.path.exists(path):
            os.mkdir(path)

    def odom_signal(self, msg):
        self.odom = msg
        self.x, self.y, self.yaw = self.convert_pose_to_xy_and_theta(self.odom.pose.pose)
        # print "self.x", self.x

    def camera_signal(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # print self.x, self.y, self.yaw
        # cv2.imshow('window_window', self.cv_image)
        # cv2.waitKey(5)

    def spin360(self):
        # first time
        if self.theta_start is None:
            self.theta_start = self.yaw
            self.theta_end = self.theta_start - 10 * math.pi / 180

        # self.theta_end = self.yaw - self.theta_start - 10 * math.pi / 180
        # if self.about_equal(self.theta_end, 0):
        if self.about_equal(self.yaw, self.theta_end):
            self.twist.angular.z = 0
            # self.theta_start = None
        else:
            self.twist.angular.z = self.angularVelocity
        self.pub.publish(self.twist)

    @staticmethod
    def about_equal(x, y, epsilon=0.15):
        val = abs(x - y)
        return val < epsilon

    @staticmethod
    def smaller_about_equal(x, y, epsilon=.5):
        val = abs(x - y)
        return val < epsilon


    def screenshot(self):
        if self.smaller_about_equal(abs(self.yaw)%self.diff, 0) and self.cv_image is not None:
            # SAVE PHOTOS
            degrees = 180/math.pi * self.yaw
            print degrees
            if abs(degrees-self.lastDegrees)>=(self.diff/math.pi)*180-2.85:
                cv2.imwrite(os.path.join(self.save_path,"{}.jpg".format(int(round(degrees, 0))+180)), self.cv_image)
                self.lastDegrees = degrees
           
            # waypoint = "{theta}".format(theta=self.yaw)
            # self.camera_roll[waypoint] = self.cv_image
            # self.cv_image

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.spin360()
            self.screenshot()
            r.sleep()

if __name__ == '__main__':
    base_path = "/home/zhecan/github/Panorama_Construction/street_view_images"
    node = StreetViewDriver("/camera/image_raw", 60, base_path)
    node.run()
            # self.save_path = path

    # create_pose_folder(2.55, -3.5)