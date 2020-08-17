#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import random

from geometry_msgs.msg import Twist

import tf

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs


from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2


class QrReader(object):

    def __init__(self):
        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # camera subscriver
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.imageCallback, queue_size=1)

        # publish qr_val
        self.qr_val_pub = rospy.Publisher('qr_val', String, queue_size=1)

        # publish marked qr area image
        self.qr_img_pub = rospy.Publisher('qr_image', Image, queue_size=1)

    # comvert image topic to opencv object and show
    def imageCallback(self, data):
        try:
            im = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # read AR code
        aruco = cv2.aruco
        dictionary = aruco.getPredefinedDictionary(aruco.DICT_7X7_50)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(im, dictionary)
        aruco.drawDetectedMarkers(im, corners, ids)
        if ids is not None:
            for i in ids:
                self.qr_val_pub.publish(str(i[0]))
                print str(i[0])

        im_msg = self.bridge.cv2_to_imgmsg(im, "bgr8")
        self.qr_img_pub.publish(im_msg)

class testBot():
    def __init__(self):
        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)


    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        

    def calcTwist(self,x,th,run_time):

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        
        return twist

    def strategy(self):
        r = rospy.Rate(50) # change speed 50fps

        #move left side
        self.setGoal(-0.84,-0.51,0)
        
        self.setGoal(-0.34,0.02,0)
        
        self.setGoal(-0.9,0.52,0)
        
        self.setGoal(-0.37,0.51,0)
        self.setGoal(-0.37,0.51,3.1415/4)
        
        self.setGoal(-0.84,0.31,3.1415/2)
        
        #scan QR
        print "start QR read"
        rospy.spin()
        print "end QR read"

        #move right side
        self.setGoal(-0.34,0.02,0)

        self.setGoal(0.84,-0.51,0)
        
        self.setGoal(0.34,0.02,0)
        
        self.setGoal(0.9,0.52,0)
        
        self.setGoal(0.37,0.51,0)
        self.setGoal(0.37,0.51,-3.1415/4)
        
        self.setGoal(0.84,0.31,-3.1415/2)

        #twist = self.calcTwist(0.5,0,3)
        #self.vel_pub.publish(twist)
        #rospy.sleep(0.5)

        #twist = self.calcTwist(0,-0.1,0)
        #self.vel_pub.publish(twist)

        #scan QR
        print "start QR read"
        rospy.spin()
        print "end QR read"

        #self.setGoal(-0.6,-0.8,0)
        #self.setGoal(-0.6,-0.8,3.1415/2)

       # self.setGoal(0,0,3.1415/2)
#        target_speed = 0
#        target_turn = 0
#        control_speed = 0
#        control_turn = 0
#
#        while not rospy.is_shutdown():
#            twist = self.calcTwist()
#            print(twist)
#            self.vel_pub.publish(twist)
#
#            r.sleep()


if __name__ == '__main__':
    rospy.init_node('test_run')
    bot = testBot()
    qr = QrReader()
    bot.strategy()

