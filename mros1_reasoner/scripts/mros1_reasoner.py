#!/usr/bin/env python

import rospy
import actionlib

from cheops_system_state_msgs.msg import SystemState
from cheops_graph_manipulation_msgs.msg \
    import GraphManipulationActionAction,  GraphManipulationActionGoal, \
    GraphManipulationMessage

from collections import defaultdict

sys_state=None
graph_manipulation_client=None
last_configuration=["cs_yumi_2"] 

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



def send_request (reconfiguration_request):
    goal = GraphManipulationActionGoal()
    goal.request = reconfiguration_request
    graph_manipulation_client.send_goal(goal)
    graph_manipulation_client.wait_for_result()
    return graph_manipulation_client.get_result().result


spec2request = defaultdict(lambda: GraphManipulationMessage.REQUEST_MISSING, {
    "cs_yumi1" : GraphManipulationMessage.REQUEST_YUMI_CONFIG_1ARM,
    "cs_displacement_node" : GraphManipulationMessage.REQUEST_DISPLACEMENT_NODE,
    "cs_tag_calibration_node" : GraphManipulationMessage.REQUEST_TAG_CALIBRATION_NODE,
    "cs_camera_1" : GraphManipulationMessage.REQUEST_CAMERA_CONFIG1,
    "cs_camera_2" : GraphManipulationMessage.REQUEST_CAMERA_CONFIG2,
    "cs_tag_detector" : GraphManipulationMessage.REQUEST_TAG_DETECTOR,
    "safe_shutdown" : GraphManipulationMessage.REQUEST_SAFE_SHUTDOWN
})


def request_reconfiguration(component_specs):
    global last_configuration, spec2request
    new_specs = filter(lambda cs: cs not in last_configuration, component_specs)
    requests = map(lambda cs: spec2request[cs], new_specs)

    for r in requests:
        result = send_request(r)
        if (result != GraphManipulationMessage.RECONFIGURATION_OK):
            last_configuration = []
            return result

    last_configuration = component_specs
    return GraphManipulationMessage.RECONFIGURATION_OK


if __name__ == '__main__':
    rospy.init_node('mros1_reasoner')

    sub = rospy.Subscriber('system_state', SystemState, callback)

    timer = rospy.Timer(rospy.Duration(1.), timer_cb)

    graph_manipulation_client = actionlib.SimpleActionClient(
            'cheops_graph_manipulation_action_server', 
            GraphManipulationActionAction)
    graph_manipulation_client.wait_for_server()

    request_reconfiguration (["cs_displacement_node", "cs_tag_calibration_node", 
        "cs_camera_1", "cs_camera_2", "cs_tag_calibration_node",
        "cs_tag_detector"])

    rospy.spin()
