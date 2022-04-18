#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion
from actionlib_msgs.msg import*
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

class Many_spot():
    def __init__(self):
        rospy.init_node("points_node", anonymous=False)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_CB)
        self.ac=actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.target=MoveBaseGoal()
        self.reach_data = "empty"
        self.set_flag=True
        self.i=5
        rospy.Timer(rospy.Duration(1.0), self.move_CB)
        rospy.spin()
    
    def reach_CB(self, data):
        self.reach_data = data.status.text
        
    def move_CB(self, event):
        self.lst=["red", "orange", "yellow", "green", "blue", "navy","purple"]
        self.each_spot(self.lst[self.i])
        if self.reach_data == "Goal reached.":
            self.set_flag=True
            self.i=(self.i+1)%7
            self.reach_data = "empty"
            rospy.loginfo("Reached at Goal")
    
    def each_spot(self, color):
        if color=="red":
            self.red()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="orange":
            self.orange()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="yellow":
            self.yellow()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="green":
            self.green()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="blue":
            self.blue()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="navy":
            self.navy()
            if self.set_flag==True:
                self.ac.send_goal(self.target)
                self.set_flag=False
        elif color=="purple":
            self.purple()
            if self.set_flag == True:
                self.ac.send_goal(self.target)
                self.set_flag=False
                
    def red(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(27.393040, -3.757976, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.406310, 0.913735)
	
    def orange(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(22.116351, -0.946563, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.999978, 0.0064848)
	
    def yellow(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(18.980420, 5.322855, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.949231, 0.314579)
	
    def green(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(4.439737, 15.667051, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, -0.344222, 0.938888)
	
    def blue(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(13.266335, 1.994368, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, -0.716580, 0.697504)
    
    def navy(self):
        self.target.target_pose.header.stamp=rospy.Time.now()
        self.target.target_pose.header.frame_id="map"
        self.target.target_pose.pose.position=Point(19.790006, -10.629376, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.187867, 0.982194)
    
    def purple(self):
        self.target.target_pose.header.stamp = rospy.Time.now()
        self.target.target_pose.header.frame_id = "map"
        self.target.target_pose.pose.position = Point(19.228476,-7.499958, 0.0)
        self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.249728, 0.968315)


if __name__ == '__main__':
    Many_spot()