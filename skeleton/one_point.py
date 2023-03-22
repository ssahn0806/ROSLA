#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import Point, Quaternion
from actionlib_msgs.msg import*
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult

class Many_spot():
	def __init__(self):
		rospy.init_node("point_node", anonymous=False)
		rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.reach_CB)
		self.ac=actionlib.SimpleActionClient("move_base", MoveBaseAction)
		self.target=MoveBaseGoal()
		self.reach_data = "empty"
		self.set_flag=True
		self.i=0
		rospy.Timer(rospy.Duration(1.0), self.move_CB)
		rospy.spin()

	def reach_CB(self, data):
		self.reach_data = data.status.text

	def move_CB(self, event):
		self.lst=["red"]
		self.each_spot(self.lst[self.i])
		if self.reach_data == "Goal reached.":
			self.set_flag=True
			self.i=(self.i+1)%1
			self.reach_data = "empty"
			rospy.loginfo("Reached at Goal")
			

	def each_spot(self, color):
		if color=="red":
			self.red()
			if self.set_flag==True:
				self.ac.send_goal(self.target)
				self.set_flag=False


	def red(self):
		self.target.target_pose.header.stamp=rospy.Time.now()
		self.target.target_pose.header.frame_id="map"
		self.target.target_pose.pose.position=Point(27.393040, -3.757976, 0.0)
		self.target.target_pose.pose.orientation=Quaternion(0, 0, 0.406310, 0.913735)


if __name__ == '__main__':
	Many_spot()