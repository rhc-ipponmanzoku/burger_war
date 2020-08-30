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

import json

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

        # score subscriver
        self.myColor     = None
        self.myScore     = 0
        self.enemyScore  = None
        self.warState    = None
        self.score_sub   = rospy.Subscriber('/war_state', String, self.warStateCallback, queue_size=1)

        ############
        ##   QR   ##
        ############
        # for convert image topic to opencv obj
        self.bridge = CvBridge()

        # camera subscriver
        self.image_sub = rospy.Subscriber('/image_raw', Image, self.imageCallback, queue_size=1)

        # publish qr_val
        self.qr_val_pub = rospy.Publisher('qr_val', String, queue_size=1)

        # publish marked qr area image
        self.qr_img_pub = rospy.Publisher('qr_image', Image, queue_size=1)

    #warState callback
    def warStateCallback(self,data):
        warState = data
        jsonWarState = json.loads(warState.data)
        self.warState = jsonWarState["scores"]
        print self.warState

        # which team?
        if jsonWarState["players"]["r"] == "you":
            #print " you are red"
            self.myColor = "r"
        else:
            #print " you are blue"
            self.myColor = "b"

        #update myScore
        self.myScore = jsonWarState["scores"][self.myColor]
        print self.myScore

        #print jsonWarState["players"]["r"]
        #print self.warState["b"]
        #self.warState = jsonWarState.keys()

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
#        wait = self.client.wait_for_result()
#        if not wait:
#            rospy.logerr("Action server not available!")
#            rospy.signal_shutdown("Action server not available!")
#        else:
#            return self.client.get_result()        

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
            print "find ID!!!"
            for i in ids:
                self.qr_val_pub.publish(str(i[0]))
                print str(i[0])

                print next_point
                if next_point < [len(v) for v in goal_point][0] :
                    state = "move_goal"
                    next_point = next_point + 1


        im_msg = self.bridge.cv2_to_imgmsg(im, "bgr8")
        self.qr_img_pub.publish(im_msg)

    def calcTwist(self,x,th,run_time):

        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        
        return twist

    def strategy(self):
        r = rospy.Rate(10) # change speed 10fps

        goal_point = [ [-0.84,-0.51],#first(x,y)
                       [-0.47,0.02],#second(x,y)
                       [ -0.9,0.52], #third(x,y)
                       [-0.37,0.51] 
                      ]
        next_spot = 0
        now_score = 0
        state = "move_goal"

        while not rospy.is_shutdown(): 
            if state == "move_goal":
                self.setGoal(goal_point[next_spot][0],
                             goal_point[next_spot][1],
                             0)
                print "next_spot : " + str(next_spot)
                state = "wait_get_point"
                print "waiting"
            elif state == "wait_get_point":
                state = "wait_get_point"

                #reach target goal  or
                if (now_score+1) == int(self.myScore):
                    now_score = int(self.myScore)
                    print "Next Spot !!!!!!"
                    state = "move_goal"
                    if next_spot < len(goal_point):# [len(v) for v in goal_point][0]:
                        next_spot = next_spot + 1
                        self.client.cancel_goal()

        
#        self.setGoal(-0.34,0.02,0)
        
#        self.setGoal(-0.9,0.52,0)
        
#        self.setGoal(-0.37,0.51,0)
#        self.setGoal(-0.37,0.51,3.1415/4)
  
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
#    qr = QrReader()
    bot.strategy()

