#!/usr/bin/env python

import rospy

class Node():
    def __init__(self):
        self.current = rospy.get_param('/move_base/DWAPlannerROS/yaw_goal_tolerance')
        rospy.Timer(rospy.Duration(5.0),self.change)
    def change(self,event):
        rospy.loginfo('current value: {}'.format(self.current))
        rospy.set_param('/move_base/DWAPlannerROS/yaw_goal_tolerance',3.14)
        


def run():
    rospy.init_node('change_param_node')
    Node()
    rospy.spin()

if __name__ == '__main__':
    run()