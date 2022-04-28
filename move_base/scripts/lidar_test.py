#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class MoveSimulator:
    def __init__(self):
        rospy.init_node('problem_7')
        rospy.Subscriber('/scan',LaserScan,self.subTopic)
        self.pubVel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        rospy.spin()
    def subTopic(self,data):
        movement = Twist()
		# if self.shouldStop(data.ranges[150:210]):
		# if self.shouldStop(data.ranges[60:120]):        
        # if self.shouldStop(data.ranges[480:672]):
        # 3.2
        # 150 ~ 210
        # 165 ~ 195
        if self.shouldStop(data.ranges[512:640]):
            movement.linear.x = 0.0
            movement.angular.z = 0.0
            rospy.loginfo('turn left')
        else :
            movement.linear.x = 1.0
            rospy.loginfo('go forward')
        # rospy.loginfo('len : {}'.format(len(data.ranges)))
        self.pubMsg(movement)
    def shouldStop(self,ranges):
        for i in ranges:
            if i!= 'inf' and i<= 1.0:
                return True
        return False
    def pubMsg(self,target):
        self.pubVel.publish(target)

if __name__ == '__main__':
    MoveSimulator()
