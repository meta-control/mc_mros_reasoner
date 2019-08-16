#!/usr/bin/env python

import rospy

from cheops_system_state_msgs.msg import SystemState


sys_state=None
def callback(msg):
    global sys_state
    sys_state = msg


def timer_cb(event):
    global sys_state

    if not sys_state:
        rospy.logwarn_throttle(1., 'Still waiting on system state ..')
        return

    rospy.loginfo_throttle(1., 'camera: {}, tag_detect: {}, yumi: {}'.format(
        sys_state.camera_status, sys_state.tag_detection_status, sys_state.yumi_status))

    # TODO: update reasoner facts, evaluate, retrieve action, publish


if __name__ == '__main__':
    rospy.init_node('mros1_reasoner')

    sub = rospy.Subscriber('system_state', SystemState, callback)

    timer = rospy.Timer(rospy.Duration(1.), timer_cb)

    rospy.spin()
