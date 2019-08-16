#!/usr/bin/env python

import rospy

import actionlib

from cheops_system_state_msgs.msg import SystemState
from cheops_graph_manipulation_msgs.msg import GraphManipulationActionAction

sys_state=None
graph_manipulation_client=None

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

def request_reconfiguration(component_specs):

    return


if __name__ == '__main__':
    rospy.init_node('mros1_reasoner')

    sub = rospy.Subscriber('system_state', SystemState, callback)

    timer = rospy.Timer(rospy.Duration(1.), timer_cb)

    graph_manipulation_client = actionlib.SimpleActionClient(
            'cheops_graph_manipulation_action_server', 
            GraphManipulationActionAction)
    graph_manipulation_client.wait_for_server()

    rospy.spin()
