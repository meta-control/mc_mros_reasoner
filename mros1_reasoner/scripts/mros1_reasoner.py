#!/usr/bin/env python

import rospy

if __name__ == '__main__':
    rospy.init_node('mros1_reasoner', log_level=rospy.DEBUG)
    rospy.loginfo("It worked!")
