#!/usr/bin/env python
#-*- coding:utf-8 -*-

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

'''
조이스틱 혹은 teleopkey를 통해서 cmd_vel 토픽을 발행하는 클래스

- 전진
- 후진
- 회전(반시계)
- 회전(시계)

'''
class Controller:
    def __init__(self):
        # -1 : 정지, 0 : 전진, 1 : 후진, 2: 반시계 회전, 3: 시계 회전
        self.current_state = -1
        
        self.dist_threshold = 1.5
        self.yaw_tolerance = 15.0
        
        self.current_point = Point()
        self.current_yaw = 0.0
        
        self.target_state = -1
        self.target_yaw = 0.0
        
        rospy.Subscriber('/scan',LaserScan,self.scan_CB)
        rospy.Subscriber('/odom',Odometry,self.odom_CB)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        rospy.Timer(rospy.Duration(0.1),self.control)
        
    def scan_CB(self,data):
        # 거리가 1.5m 이상이면서 가장 긴 곳을 방향으로 택한다.
        max_idx,max_val = -1,data.range_min-1
        
        for index,range in enumerate(data.ranges):
            if range > max_val and range >=self.dist_threshold:
                max_idx = index
                max_val = range
        
        self.target_yaw = (data.angle_min + max_idx * data.angle_increment)*180.0/math.pi
        
    def odom_CB(self,data):
        # 현재 위치,각도 정보 업데이트
        pose = data.pose.pose.position
        orient = data.pose.pose.orientation
        
        self.current_point = pose
        _,_,self.current_yaw = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        self.current_yaw = self.current_yaw*180.0/math.pi   
        
    def control(self,event):
        # 주기적으로 cmd_vel 명령을 내리는 메서드
        
        # 내가 지정한 각도와 차이가 tolerance 이상이라면 회전 명령
        if abs(self.target_yaw - self.current_yaw) > self.yaw_tolerance:
            self.turn_left()
            rospy.loginfo('[Turning] target : {}, current : {}'.format(self.target_yaw,self.current_yaw))
        # 각도가 어느정도 맞으면 전진 명령
        else :
            self.go_forward()
            rospy.loginfo('[Forwarding] target : {}, current : {}'.format(self.target_yaw,self.current_yaw))
            
    
    def go_forward(self):
        # 전진
        pub_data = Twist()
        pub_data.linear.x = 0.5
        pub_data.linear.y = 0
        pub_data.linear.z = 0
        
        pub_data.angular.x = 0
        pub_data.angular.y = 0
        pub_data.angular.z = 0
        
        self.pubTopic(pub_data)
        
    def go_backward(self):
        # 후진
        pub_data = Twist()
        pub_data.linear.x = -1.0
        pub_data.linear.y = 0
        pub_data.linear.z = 0
        
        pub_data.angular.x = 0
        pub_data.angular.y = 0
        pub_data.angular.z = 0
        
        self.pubTopic(pub_data)
        
    def turn_left(self):
        # 반시계 회전
        pub_data = Twist()
        pub_data.linear.x = 0
        pub_data.linear.y = 0
        pub_data.linear.z = 0
        
        pub_data.angular.x = 0
        pub_data.angular.y = 0
        pub_data.angular.z = -0.5
        
        self.pubTopic(pub_data)
        
    def turn_right(self):
        # 시계 회전
        pub_data = Twist()
        pub_data.linear.x = 0
        pub_data.linear.y = 0
        pub_data.linear.z = 0
        
        pub_data.angular.x = 0
        pub_data.angular.y = 0
        pub_data.angular.z = 0.5
        
        self.pubTopic(pub_data)
            
    def pubTopic(self,data):
        # cmd_vel 발행
        self.cmd_pub.publish(data)



def run():
    rospy.init_node('controller_node')
    Controller()
    rospy.spin()

if __name__ == '__main__':
    run()
        
    