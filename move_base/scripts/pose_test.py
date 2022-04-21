#!/usr/bin/env python

import rospy

from geometry_msgs.msg import PolygonStamped,PoseWithCovarianceStamped

class Checker():
    def __init__(self):
        rospy.init_node('pose_check_node')
        rospy.Subscriber('/amcl_pose',PoseWithCovarianceStamped,self.amcl_CB)
        rospy.Subscriber('/move_base/global_costmap/footprint',PolygonStamped,self.global_CB)
        # rospy.Subscriber('/move_base/local_costmap/footprint',PolygonStamped,self.local_CB)
        rospy.spin()

    def amcl_CB(self,data):
        pose = data.pose.pose.position
        orientation = data.pose.pose.orientation
        rospy.loginfo('amcl : {} {}'.format(pose,orientation))
    
    def global_CB(self,data):
        points = data.polygon.points

        cx,cy = 0,0
        for p in points:
            cx += p.x
            cy += p.y
        
        cx /=4
        cy /=4

        rospy.loginfo('global footprint : {} {}'.format(cx,cy))
    
    def local_CB(self,data):
        points = data.polygon.points

        cx,cy = 0,0
        for p in points:
            cx += p.x
            cy += p.y
        
        cx /=4
        cy /=4

        rospy.loginfo('local footprint : {} {}'.format(cx,cy))

def run():
    Checker()

if __name__ == '__main__':
    run()