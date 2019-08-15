#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('mros1_reasoner', log_level=rospy.DEBUG)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.loginfo_throttle(1, "Reasoner node running (dummy) ..")
        rate.sleep()
