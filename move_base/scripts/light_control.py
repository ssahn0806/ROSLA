#!/usr/bin/env python

import rospy
from scout_msgs.msg import ScoutLightCmd,ScoutStatus

class LightController:
    def __init__(self):
        self.light_pub = rospy.Publisher('/scout_light_control',ScoutLightCmd,queue_size=1)
        self.current_mode = 0
        rospy.Subscriber('/scout_status',ScoutStatus,self.status_CB)

        rospy.Timer(rospy.Duration(0.1),self.control)

    def control(self,event):
        new_mode = ScoutLightCmd()
        new_mode.enable_cmd_light_control = True
        new_mode.front_mode = 0
        # new_mode.rear_mode = 0
        rospy.loginfo('now front mode {}'.format(new_mode.front_mode))
        self.light_pub.publish(new_mode)
    
    def status_CB(self,data):
        self.current_mode = data.front_light_state.mode

def run():
    rospy.init_node('light_control_node')
    LightController()
    rospy.spin()

if __name__ == '__main__':
    run()