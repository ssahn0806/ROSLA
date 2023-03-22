#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Point,PoseStamped,PoseWithCovarianceStamped,Quaternion
from move_base_msgs.msg import MoveBaseActionResult
from actionlib_msgs.msg import GoalID,GoalStatusArray


class Navigator:
	def __init__(self):
		self.goal_pos = Point()
		self.cur_pos = Point()
		self.goal_idx = 0
		self.is_finish = True
		self.THRESHOLD = 0.3
		self.goal_pos_lst = [(6.283474,2.278948,0.0),(3.216741,6.636334,0.0),(0.635803,17.714057,0.0),(-3.177226,14.713905,0.0),(8.486019,-12.203601,0.0),(14.557521,-4.168218,0.0),(12.561250,4.213137,0.0)]
		self.goal_ori_lst = [(0.0,0.0,0.991320,-0.131466),(0.0,0.0,0.691378,0.722492),(0.0,0.0,-0.629272,0.777185),(0.0,0.0,0.826651,0.562714),(0.0,0.0,-0.433854,0.900982),(0.0,0.0,-0.989306,0.145851),(0.0,0.0,0.695757,0.718276)]		
		
		
		self.goal_id = ''

		self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
		self.finish_pub = rospy.Publisher('/move_base/cancel',GoalID,queue_size=1)
		
		rospy.Timer(rospy.Duration(1.0), self.pubGoal) ## 1hz

		rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.pose_CB)
		rospy.Subscriber('/move_base/status',GoalStatusArray,self.status_CB)
		# rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.parking_CB)


	# callback when new 2d pose goal published
	def goal_CB(self,data):

		if not data:
			self.goal_pos = Point(self.goal_pub_lst[0][0],self.goal_pub_lst[0][1],self.goal_pub_lst[0][2])
			self.goal_idx = 0
			rospy.loginfo('first goal pose : [{}] ({},{},{})'.format(self.goal_idx+1,self.goal_pos.x,self.goal_pos.y,self.goal_pos.z))
		else :
			self.goal_pos = Point(data.pose.position.x,data.pose.position.y,data.pose.position.z)
			self.goal_idx = self.searchIdx(self.goal_pos)
			# if self.goal_idx == 3:
			# 	self.THRESHOLD = 0.3
			# elif self.goal_idx == 6:
			# 	self.THRESHOLD = 0.01
			# else : 
			# 	self.THRESHOLD = 0.5
			rospy.loginfo('current goal pose : [{}] ({},{},{})'.format(self.goal_idx+1,self.goal_pos.x,self.goal_pos.y,self.goal_pos.z))


	def searchIdx(self,target):
		for idx,pose in enumerate(self.goal_pos_lst):
			if pose[0] == target.x and pose[1] == target.y and pose[2] == target.z:
				return idx	
		else :
			return -1

	# callback when current robot's position changed
	def pose_CB(self,data):
		self.cur_pos = data.pose.pose.position
		
		if self.check_finish(self.goal_pos,self.cur_pos):
			if self.goal_idx < 7:
				if self.goal_idx == 0:
					rospy.loginfo('Initialize for Path Following...')
				else :
					rospy.loginfo('goal {} finished!'.format(self.goal_idx))
				self.is_finish = True
			else :
				if not self.is_finish:
					rospy.loginfo('Path Following Finished for 7 Goal Points')
					self.pubFinish()
	# def parking_CB(self,data):

	# calculate difference between goal and pose - return True,False
	def check_finish(self,goal,current):
		return ((goal.x - current.x)**2  + (goal.y - current.y)**2)**.5 < self.THRESHOLD

	def status_CB(self,data):
		if len(data.status_list):
			self.goal_id = data.status_list[0].goal_id.id

	def pubFinish(self):
		# finish_data = MoveBaseActionResult()
		# finish_data.header.stamp = rospy.Time.now()
		# finish_data.header.frame_id = ''
		# finish_data.status.status = 3
		# finish_data.status.goal_id.id = self.goal_id
		# finish_data.status.goal_id.stamp = rospy.Time.now()
		# finish_data.status.text = "Goal reached."

		finish_data = GoalID()
		finish_data.stamp = rospy.Time.now()
		finish_data.id = self.goal_id

		# finish_data = PoseStamped()
		# finish_data.header.stamp = rospy.Time.now()
		# finish_data.header.frame_id = "map"

		# finish_data.pose.position = self.cur_pos
		# finish_data.pose.orientation.x = 0.0
		# finish_data.pose.orientation.y = 0.0
		# finish_data.pose.orientation.z = 0.0
		# finish_data.pose.orientation.w = 1.0
		self.is_finish = True
		self.finish_pub.publish(finish_data)

	# current goal is finished, new goal publishing
	def pubGoal(self,event):

		if self.goal_idx < len(self.goal_pos_lst) and self.is_finish:
			publish_data = PoseStamped()
			publish_data.header.stamp = rospy.Time.now()
			publish_data.header.frame_id = "map"

			publish_data.pose.position = Point(self.goal_pos_lst[self.goal_idx][0],self.goal_pos_lst[self.goal_idx][1],self.goal_pos_lst[self.goal_idx][2])
			# publish_data.pose.orientation = Quaternion(self.goal_ori_lst[self.goal_idx][0],self.goal_ori_lst[self.goal_idx][1],self.goal_ori_lst[self.goal_idx][2],self.goal_ori_lst[self.goal_idx][3])
			publish_data.pose.orientation.x = self.goal_ori_lst[self.goal_idx][0]
			publish_data.pose.orientation.y = self.goal_ori_lst[self.goal_idx][1]
			publish_data.pose.orientation.z = self.goal_ori_lst[self.goal_idx][2]
			publish_data.pose.orientation.w = self.goal_ori_lst[self.goal_idx][3]

			self.goal_idx +=1
			self.goal_pos = publish_data.pose.position
			rospy.loginfo('new Goal {} Publish'.format(self.goal_idx))
			self.is_finish = False

			self.goal_pub.publish(publish_data)


def run():
	rospy.init_node('navigate_node')
	Navigator()
	rospy.spin()

if __name__ == '__main__':
	run()