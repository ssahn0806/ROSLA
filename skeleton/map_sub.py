#!/usr/bin/env python


class MapSubscriber:
    def __init__(self):
        self.width = 0
        self.height = 0

        self.grid = None

        rospy.
    def map_CB(self,data):


def run():
    rospy.init_node("map_subscriber_node")
    MapSubscriber()
    rospy.spin()

if __name__ == '__main__':
    run()