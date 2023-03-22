#!/usr/bin/env python
#-*- coding:utf-8 -*-

from tabnanny import check
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
        self.dist_threshold = 1.0
        self.yaw_tolerance = 15.0
        
        self.current_point = Point()
        self.current_yaw = 0.0
        
        self.target_yaw = 0.0
            
        self.has_target = False
        
        self.should_left = False
        self.should_right = False
        self.should_stop = False
        
        rospy.Subscriber('/scan',LaserScan,self.scan_CB)
        rospy.Subscriber('/odom',Odometry,self.odom_CB)
        
        self.cmd_pub = rospy.Publisher('/cmd_vel',Twist,queue_size=1)

        rospy.Timer(rospy.Duration(0.1),self.control)
                   
    def check_stop(self,ranges):
        for i in ranges:
            if i!='inf' and i<=self.dist_threshold:
                return True
        return False
     
    def scan_CB(self,data):
        # 거리가 1.5m 이상이면서 가장 긴 곳을 방향으로 택한다.
        max_idx,max_val = -1,data.range_min-1
        
        for index,range in enumerate(data.ranges):
            if range > max_val and range >=self.dist_threshold:
                max_idx = index
                max_val = range
        
        if not self.has_target:
            self.target_yaw = (data.angle_min + max_idx * data.angle_increment)*180.0/math.pi
            self.has_target = True
            self.should_stop = False
        
        # # 우측 라이다가 1.5m이내에 찍히는 점이 발생하면 왼쪽으로 회전해야함
        # if self.check_stop(data.ranges[64:512]): # 2-160
        #     self.should_left = True
        # else :
        #     self.should_left = False
            
        # # 좌측 라이다가 1.5m이내에 찍히는 점이 발생하면 오른쪽으로 회전해야함
        # if self.check_stop(data.ranges[640:1088]): # 200 ~ 340
        #     self.should_right = True
        # else :
        #     self.should_right = False
            
        # 전방 라이다가 1.5m이내에 찍히는 점이 발생하면 정지해야함 
        if self.check_stop(data.ranges[512:640]): # 160~200
            self.should_stop = True

        
    def odom_CB(self,data):
        # 현재 위치,각도 정보 업데이트
        pose = data.pose.pose.position
        orient = data.pose.pose.orientation
        
        self.current_point = pose
        _,_,self.current_yaw = euler_from_quaternion([orient.x,orient.y,orient.z,orient.w])
        self.current_yaw = self.current_yaw*180.0/math.pi   
        
    def control(self,event):
        # 주기적으로 cmd_vel 명령을 내리는 메서드
        
        # 가장 멀리 갈 수 있는 방향으로 로봇 방향 맞추기
        if self.target_yaw - self.current_yaw > self.yaw_tolerance:
            self.turn_right()
            rospy.loginfo('[Turn Left] target : {}, current : {}'.format(self.target_yaw,self.current_yaw))
        elif self.target_yaw - self.current_yaw < -self.yaw_tolerance:
            self.turn_left()
            rospy.loginfo('[Turn Right] target : {}, current : {}'.format(self.target_yaw,self.current_yaw)) 
        else :
            self.should_left = False
            self.should_right = False
            # 전방에 막히는 것이 없다면 전진
            if not self.should_stop:
                self.go_forward()         
                # 전진하면서 좌,우측에 가까워지면 회전명령
                if self.should_left:
                    self.turn_left()
                if self.should_right:
                    self.turn_right()
            # 더이상 전진할 수 없다면 - 새로운 방향을 찾도록 target을 해제한다.
            else :
                self.has_target = False
            
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
        pub_data.linear.x = -0.5
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
        
    