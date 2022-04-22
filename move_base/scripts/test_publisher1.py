#!/usr/bin/env python
#-*- coding:utf-8 -*-

'''
본 코드가 수행해야 하는 일

1. 현재 목표지점에 도달했는지 여부를 확인
2. 도달했다면 다음 목표지점을 발행

[고려해야할 사항]
1. 도달여부를 무엇으로 판단할 것인가

- 목표위치와 현재위치간 거리를 계산하여 일정 범위 이내로 판단 (정확성 떨어짐)
- 목표에 도달한 상태를 발행하는 토픽의 결과로 판단 (정확성 높음)

- 거리만 고려하게 되면 자세(Orientation)를 고려하지 않아서 부정확하고, 토픽의 결과를 활용하면 너무 정확하게 맞을 떄 까지(Tolerance) 현재 목표를 추적한다.
- 중간지점이냐, 최종지점이냐 또는 자세가 중요하게 고려되는 지점이냐에 따라 선택할 것

'''

import sys
import rospy
import actionlib
import tf

from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, PolygonStamped # 2D Nav Goal(PoseStamped), amcl_pose(PoseWithCovarianceStamped) Topic
from actionlib_msgs.msg import GoalID,GoalStatusArray # move_base/cancel(GoalID), move_base/status(GoalStatusArray) Topic
from move_base_msgs.msg import MoveBaseAction,MoveBaseGoal,MoveBaseActionResult # move_base msg

from scout_msgs.msg import ScoutStatus,ScoutLightCmd # scout_light_control(ScoutLightCmd) Topic
from std_srvs.srv import Empty # clear_costmap service




class clearService:
    def __init__(self):
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    def request(self):
        try :
            self.client()
            rospy.loginfo('service_call_granted')
        except rospy.ServiceException as e:
            rospy.loginfo('service_call_failed' + e)
            
        
class GoalPubNode:
    def __init__(self):
        self.cur_goal = Pose()
        self.cur_pose = Pose()
        
        self.goal_lst = []
        self.ori_lst = []
        
        self.idx = -1
        self.goal_id = ''
        self.is_stop = True
        self.is_reach = False
        self.is_finish = False
        
        self.LENGTH = 0
        self.dist_TH = 0.1
        self.deg_TH = 5
        
        self.goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=5)
        
        self.action = actionlib.SimpleActionClient('move_base',MoveBaseAction)        
        
        self.cancel_pub = rospy.Publisher('/move_base/cancel',GoalID,queue_size=3)
        self.light_pub = rospy.Publisher('/scout_light_control',ScoutLightCmd,queue_size=1)
        
        self.rate = rospy.Rate(0.1) # 1 per 10s
        
        self.service = clearService()
        
        self.change_light_mode(1)

        self.init_goal_lst()

        rospy.Timer(rospy.Duration(1.0),self.pub_new_goal_1)
        # rospy.Timer(rospy.Duration(10.0),self.clear_request)
        # rospy.Timer(rospy.Duration(1.0),self.pub_newgoal_2)
        
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.current_CB)
        rospy.Subscriber('/move_base/global_costmap/footprint',PolygonStamped,self.current_CB2)
        rospy.Subscriber('/move_base_simple/goal',PoseStamped,self.print_new_goal)
        rospy.Subscriber('/move_base/status',GoalStatusArray,self.status_CB)
        rospy.Subscriber('/move_base/result',MoveBaseActionResult,self.result_CB)
        # rospy.Subscriber('/scout_status',ScoutStatus,self.scout_CB)
        pass
    
    # 도착해야 할 지점들의 위치 순서대로 등록(x,y,z)
    def init_goal_lst(self):
        # 1st parking
        self.goal_lst.append(Point(0.8026,11.9794,0.0)) # 0.8202, 11.9073

        # 2nd parking
        self.goal_lst.append(Point(-2.64,8.33,0)) # -2.57, 8.12

        # 3rd parking(1)

        self.goal_lst.append(Point(2.09,6.42,0)) # 2.50, 6.31

        # 3rd parking(2)
        # self.goal_lst.append(Point(2.87,4.87,0)) # -2.57, 8.12

        # 4rd Point
        self.goal_lst.append(Point(8.31,-17.42,0)) # 9.09, -18.78

        # 5th Point
        self.goal_lst.append(Point(15.90,-10.05,0)) # 16.29,-9.40

        # End Point
        self.goal_lst.append(Point(13.0533,-2.3557,0)) # 12.92 , -2.27
        

       
        
        self.cur_goal.position = self.goal_lst[0]  
        self.init_goal_ori()
    
    # 도착해야 할 지점들의 자세 순서대로 등록(x,y,z,w)
    def init_goal_ori(self):
        # 1st parking
        self.ori_lst.append(Quaternion(0,0,0.7865,0.6175))

        # 2nd parking
        self.ori_lst.append(Quaternion(0,0,-0.61,0.78))

        # 3rd parking(1)
        self.ori_lst.append(Quaternion(0,0,0.77,0.62))

        # 3rd parking(2)
        # self.ori_lst.append(Quaternion(0,0,-0.55,0.83))

        # 4rd Point
        self.ori_lst.append(Quaternion(0,0,0.77,0.62))

        # 5th Point
        self.ori_lst.append(Quaternion(0,0,0.19,0.98))

        # End Point
        self.ori_lst.append(Quaternion(0,0,0.883,0.9960))



        self.LENGTH = len(self.ori_lst)
        self.cur_goal.orientation = self.ori_lst[0]
        self.idx = 0
        self.service.request()

    
    # 로봇의 현재 지도 상에서 추정되는 위치가 변경되었을 때 callback(amcl_pose)
    def current_CB(self,data):
        # data : PoseWithCovarianceStamped 
        # data.pose : PoseWithCovariance
        # data.pose.pose : Pose
        
        # 현재 위치 갱신(Update)
        self.cur_pose.position = data.pose.pose.position # Point
        self.cur_pose.orientation = data.pose.pose.orientation # Quaternion
        
    def current_CB2(self,data):
        points = data.polygon.points

        cx,cy = 0,0
        for p in points:
            cx += p.x
            cy += p.y
        
        cx /=4
        cy /=4

        self.cur_pose.position = Point(cx,cy,0)    
    
    # 목표지점의 정보를 반환하는 callback(move_base/status)
    def status_CB(self,data):
        if len(data.status_list):
            self.goal_id = data.status_list[0].goal_id.id
    
    # 목표지점에 도착했음을 반환하는 토픽(move_base/result)
    def result_CB(self,data):
        if data.status.text == "Goal reached.":
            self.is_reach = True
        else :
            self.is_reach = False
    
    # def scout_CB(self,data):
    #     if self.idx > 0 and data.linear_velocity == 0 and self.is_reach == False:
    #         self.service.request()
    #     pass
    
    def clear_request(self,event):
        self.service.request()

    # 새로운 도착지점이 발행되었을 때 확인하는 callback(move_base_simple/goal)   
    def print_new_goal(self,data): # PoseStamped
        # data.pose : Pose
        pose = data.pose.position
        orient = data.pose.orientation
        
        # 새로운 도착지점 정보 출력
        rospy.loginfo('Current Goal [{}]: {}, {}'.format(self.idx+1,pose,orient))
        pass
    
    # 현재 도착지점에 도달했을 때 새로운 도착지점을 발행하는 메서드
    
    # By Topic
    def pub_new_goal_1(self,event):  
        if self.check_reach(): 

            self.change_light_mode(1)
            finish_goal = GoalID()
            finish_goal.stamp = rospy.Time.now()
            finish_goal.id = self.goal_id
            self.cancel_pub.publish(finish_goal)
            self.is_stop = True

            if self.idx == 0:                
                rospy.loginfo('sleep 4sec')
                rospy.sleep(4.0) 
                rospy.loginfo('sleep end')
                rospy.set_param('/move_base/DWAPlanerROS/yaw_goal_tolerance',1.57)
                rospy.set_param('/move_base/DWAPlanerROS/xy_goal_tolerance',0.2)
                # rospy.set_param('/move_base/DWAPlanerROS/occdist_scale',0.03)
            if self.idx == 1:
                rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance',3.14)
            elif self.idx == self.LENGTH-3:
                rospy.set_param('move_base/DWAPlannerROS/max_vel_x',1.0)
                rospy.set_param('move_base/DWAPlannerROS/min_vel_x',-1.0)

                rospy.set_param('move_base/DWAPlannerROS/max_vel_trans',1.0)
                rospy.set_param('move_base/DWAPlannerROS/min_vel_trans',-1.0)


            if self.idx == self.LENGTH-2:
                rospy.set_param('move_base/DWAPlannerROS/max_vel_x',1.5)
                rospy.set_param('move_base/DWAPlannerROS/min_vel_x',-1.5)

                rospy.set_param('move_base/DWAPlannerROS/max_vel_trans',1.5)
                rospy.set_param('move_base/DWAPlannerROS/min_vel_trans',-1.5)

                rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance',0.05)
                rospy.set_param('/move_base/DWAPlanerROS/xy_goal_tolerance',0.05)
                # rospy.set_param('/move_base/DWAPlanerROS/occdist_scale',0.02)

            self.idx +=1
            
            if self.idx < self.LENGTH:     
                new_goal = PoseStamped()
                new_goal.header.stamp = rospy.Time.now()
                new_goal.header.frame_id = 'map'
                
                new_goal.pose.position = self.goal_lst[self.idx]
                new_goal.pose.orientation = self.ori_lst[self.idx]
                
                self.cur_goal.position = new_goal.pose.position
                self.cur_goal.orientation = new_goal.pose.orientation
                
                self.is_stop = False
                self.is_reach = False

                self.goal_pub.publish(new_goal)
                self.change_light_mode(0)
                
                rospy.loginfo('New Goal {} Publish.'.format(self.idx+1))
            
            else : # 마지막 목표지점에 도달한 경우 움직임 없도록 현재 목표 취소
                if not self.is_finish:
                    finish_goal = GoalID()
                    finish_goal.stamp = rospy.Time.now()
                    finish_goal.id = self.goal_id
                    self.cancel_pub.publish(finish_goal)     
                    self.is_finish = True   
                    self.is_stop = True
                    self.is_reach = True
                    self.change_light_mode(2)
                    rospy.loginfo('Path Following End..')
                    sys.exit(0)        
        else :  
            if self.is_stop:
                new_goal = PoseStamped()
                new_goal.header.stamp = rospy.Time.now()
                new_goal.header.frame_id = 'map'
                
                new_goal.pose.position = self.goal_lst[self.idx]
                new_goal.pose.orientation = self.ori_lst[self.idx]
                
                self.cur_goal.position = new_goal.pose.position
                self.cur_goal.orientation = new_goal.pose.orientation
                
                self.is_stop = False
                self.is_reach = False

                self.goal_pub.publish(new_goal)
                self.change_light_mode(0)

        
    # By ActionClient
    def pub_new_goal_2(self,event):
        if self.check_reach():  
            self.idx +=1
            if self.idx < self.LENGTH:
                new_goal = MoveBaseGoal()
                new_goal.target_pose.header.stamp = rospy.Time.now()
                new_goal.target_pose.header.frame_id = 'map'
                
                new_goal.target_pose.pose.position = self.goal_lst[self.idx]
                new_goal.target_pose.pose.orientation = self.ori_lst[self.idx]
                
                
                self.cur_goal.position = new_goal.target_pose.pose.position
                self.cur_goal.orientation = new_goal.target_pose.pose.orientation
                
                self.is_stop = False
                self.action.send_goal(new_goal)
                rospy.loginfo('New Goal {} Publish.'.format(self.idx))
            else :
                if not self.is_finish:
                    finish_goal = GoalID()
                    finish_goal.stamp = rospy.Time.now()
                    finish_goal.id = self.goal_id
                    self.cancel_pub.publish(finish_goal)    
                    self.is_finish = True
                    self.is_stop = True
                    self.is_reach = True
                    rospy.loginfo('Path Following End..')
                    sys.exit(0)        
        else :
            new_goal = MoveBaseGoal()
            new_goal.target_pose.header.stamp = rospy.Time.now()
            new_goal.target_pose.header.frame_id = 'map'
            
            new_goal.target_pose.pose.position = self.goal_lst[self.idx]
            new_goal.target_pose.pose.orientation = self.ori_lst[self.idx]
            
            
            self.cur_goal.position = new_goal.target_pose.pose.position
            self.cur_goal.orientation = new_goal.target_pose.pose.orientation
            
            self.is_stop = False
            self.action.send_goal(new_goal)
            pass
        
    # 운행 중 혹은 도착 상태에 따라 전방 라이트 모드 변경 메서드 Only Activated in AutoMode(UP)
    def change_light_mode(self,mode):
        new_mode = ScoutLightCmd()
        new_mode.enable_cmd_light_control = True
        new_mode.front_mode = mode
        self.light_pub.publish(new_mode)
        
        # 정지해야 하는 경우 10초간 sleep
        # if new_mode.front_mode == 2:
        #     self.rate.sleep()
        # pass
    
    # 목표 도달여부를 판단하는 메서드   
    def check_reach(self):
        '''
        만약 특정 상황에서 둘 중 하나만 가지고 판단을 하려고 한다면
        if self.idx == 3 or self.idx == 6:
            use is_reach
        else :
            only chefck self.calc_distance()
            
        '''

       
        # if self.idx == 0 or self.idx == 1 or self.idx == 4:
        #     if self.is_reach or self.calc_distance() < THRESHOLD:
        #         rospy.loginfo('Finished {}.'.format(self.idx+1))
        #         self.is_stop = True
        #         self.is_reach = False
        #         self.service.request()
        #         return True
        #     else :
        #         rospy.loginfo('Going to {}.'.format(self.idx+1))
        #         return False

        # else :
        #     if self.calc_distance() < THRESHOLD:
        #         rospy.loginfo('Finished {}.'.format(self.idx+1))
        #         self.is_stop = True
        #         self.is_reach = False
        #         self.service.request()
        #         return True
        #     else :
        #         rospy.loginfo('Going to {}.'.format(self.idx+1))
        #         return False
       
        if self.is_reach or (self.calc_distance() < self.dist_TH):
            self.service.request()
            rospy.loginfo('Finished {}.'.format(self.idx+1))
            return True
        # self.change_light_mode()
        rospy.loginfo('Going to {}.'.format(self.idx+1))
        return False
        
            
    # 목표지점과 현재위치 간의 거리를 계산하는 메서드
    # xy goal tolerance 보다 작으면 계산 결과는 무의미 할 수도 있다.
    def calc_distance(self):
        return ((self.cur_goal.position.x - self.cur_pose.position.x)**2 + (self.cur_goal.position.y - self.cur_pose.position.y)**2)**0.5 
    
    def calc_degree(self):
        return abs(tf.transformations.euler_from_quaternion(self.cur_goal.orientation)-tf.transformations.euler_from_quaternion(self.cur_pose.orientation))
    

        
def run():
    rospy.init_node('goal_pub_node')
    GoalPubNode()
    rospy.spin()
    
if __name__ == '__main__':
    run()
    