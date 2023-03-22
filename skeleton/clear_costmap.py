#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty

class clearService:
    def __init__(self):
        rospy.init_node('service_node_1')
        rospy.wait_for_service('/move_base/clear_costmaps')
        self.client = rospy.ServiceProxy('/move_base/clear_costmaps',Empty)
    def request(self):
        try :
            self.client()
            rospy.loginfo('service call granted')
        except rospy.ServiceException as e:
            rospy.loginfo('service call failed ' + e)

def run():
    a = clearService()
    rate = rospy.Rate(0.2)
    
    while not rospy.is_shutdown():
        a.request()
        rate.sleep()
if __name__ == '__main__':
    run()