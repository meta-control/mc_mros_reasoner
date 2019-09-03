#!/usr/bin/env python

import rospy
import rospkg
from owlready2 import *

import actionlib

from cheops_system_state_msgs.msg import SystemState
from cheops_graph_manipulation_msgs.msg \
    import GraphManipulationActionAction,  GraphManipulationActionGoal, \
    GraphManipulationMessage

from collections import defaultdict

import argparse

from init_models import *

tomasys=None
onto=None
mock=True

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# Load ontologies
def loadOntology(file):
    onto_path.append(rospack.get_path('mc_mdl_tomasys')+'/') # local folder to search for ontologies
    onto_path.append(rospack.get_path('mc_mdl_abb')+'/') # local folder to search for ontologies
    global tomasys, onto
    tomasys = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file
    onto = get_ontology(file).load()

def obtainBestFunctionDesign(o):
    global tomasys, onto
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tomasys.FunctionDesign.instances()):
        if fd.solves == f:
            fds.append(fd)
    print("\nFunctionDesigns available for obj ", o.name, ": ",fds)
    aux = 0
    best_fd = None
    for fd in fds:
        # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
        print("FD ", fd.name, fd.fd_realisability)
        if fd.fd_realisability != False:
            # FILTER if the FD error log does NOT contain the current objective
            print("Fd: ", fd.name, "error_log: ", fd.fd_error_log)
            if not o in fd.fd_error_log:
                if fd.fd_efficacy > aux:
                    best_fd = fd
                    aux = fd.fd_efficacy
    if ( best_fd == None ):
        print("*** OPERATOR NEEDED, NO SOLUTION FOUND ***")
        return None
    else:
        print("\nBest FD available", best_fd.name)
        return best_fd

'''
To solve the given Objective, recursively Grounds the required hierarchy of
sub-Objectives and Function Groundings
'''
def groundObjective(o, cspecs):
    global tomasys, onto
    print("=> Reasoner grounding objective: ", o.name)
    fd = obtainBestFunctionDesign(o)
    if( fd == None ):
        print("*** Objective ", o.name,"cannot be realised ***")
        return ["safe_shutdown"]
    fg = tomasys.FunctionGrounding("fg_" + fd.name)
    print("Roles: ",fd.roles)
    for r in fd.roles:
        b = tomasys.Binding("b_" + r.name)
        b.binding_role = r
        fg.has_bindings = b # TODO also ad the binding_component
        cspecs.append(r.roleDef)
    for f in fd.requires:
        print("Requires: ", fd.requires)
        ob = tomasys.Objective("o_" + f.name)
        ob.typeF = f
        fg.needs.append(ob)
        groundObjective(ob, cspecs)
    return cspecs

sys_state=None
graph_manipulation_client=None
last_configuration=["cs_yumi_2"]

def callback(msg):
    global sys_state
    sys_state = msg



def timer_cb(event):
    global sys_state
    global onto
    rospy.loginfo_throttle(1., 'Entered timer_cb for metacontrol reasoning')
    # Update components statuses - TODO: update objective status too
    if not sys_state:
        rospy.logwarn_throttle(1., 'Still waiting on system state ..')
        return

    if sys_state.yumi_status != 1:
        c = onto.search_one(iri="*#c_yumi")
        c.c_status = False
        print("Yumi in ERROR")
    if sys_state.camera_status != 1:
        print("Camera in ERROR")
        c = onto.search_one(iri="*c_camera")
        c.c_status = False
    if sys_state.tag_detection_status != 1:
        print("Tag not detected")
        f = onto.search_one(iri="*f_detect_tag_poses")
        for o in list(tomasys.Objective.instances() ):
            if (o.typeF == f):
                o.o_status = "INTERNAL_ERROR"

    rospy.loginfo_throttle(1., 'camera: {}, tag_detect: {}, yumi: {}'.format(
        sys_state.camera_status, sys_state.tag_detection_status, sys_state.yumi_status))

    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)

    # PRINT system status
    print("\nComponents Statuses:")
    for i in list(tomasys.ComponentState.instances()) :
        print(i.name, i.c_status)

    print("\nBindings Statuses:")
    for i in list(tomasys.Binding.instances()) :
        print(i.name, i.b_status)

    print("\nFG Statuses:")
    for i in list(tomasys.FunctionGrounding.instances()) :
        print(i.name, i.fg_status)

    print("\nObjectives Statuses:")
    for i in list(tomasys.Objective.instances()) :
        print(i.name, i.o_status)

    print("\nCC availability:")
    for i in list(tomasys.ComponentClass.instances()) :
        print(i.name, i.cc_availability)

    print("\nFD realisability:")
    for i in list(tomasys.FunctionDesign.instances()) :
        print(i.name, i.fd_realisability)


    # evaluate and retrieve desired configuration
    # init objectives in error
    objectives_internal_error = []
    for o in list(tomasys.Objective.instances() ):
        if o.o_status == "INTERNAL_ERROR":
            objectives_internal_error.append(o)
    print("\nObjectives in error:", [o.name for o in objectives_internal_error] )
    # Ground a solution hierarchy for each root objective in error. We assume here that root_objectives do not share intermediate objectives
    cspecs = []
    for o in objectives_internal_error:
        groundObjective(o, cspecs)

    # Retrieve action and publish from cspecs
    str_specs = []
    for cs in cspecs:
        str_specs.append(cs.name)
    print("RESULT CONFIG: ", str_specs)

    if len(str_specs) != 0:
        request_reconfiguration(str_specs)



def send_request (reconfiguration_request):
    global mock
    goal = GraphManipulationActionGoal()
    goal.request = reconfiguration_request
    graph_manipulation_client.send_goal(goal)
    if mock != True:
        graph_manipulation_client.wait_for_result()
        return graph_manipulation_client.get_result().result
    else:
        return GraphManipulationMessage.RECONFIGURATION_OK


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
    rospy.logwarn_throttle(1., 'Reconfiguration requested ..')
    global last_configuration, spec2request
    new_specs = filter(lambda cs: cs not in last_configuration, component_specs)
    requests = map(lambda cs: spec2request[cs], new_specs)

    for r in requests:
        rospy.logwarn_throttle(1., 'Requesting reconfig: {}'.format(r))
        result = send_request(r)
        if (result != GraphManipulationMessage.RECONFIGURATION_OK):
            last_configuration = []
            return result

    last_configuration = component_specs
    return GraphManipulationMessage.RECONFIGURATION_OK


if __name__ == '__main__':

    onto_file = "abb_dualarm_mm_complete.owl" # TODO make a ROS parameter?
    rospy.init_node('mros1_reasoner')

    sub = rospy.Subscriber('system_state', SystemState, callback)

    loadOntology(onto_file)
    rospy.loginfo("Loaded ontology: " + onto_file)
    # init specific application model using the corresponding init sript
    init_abb_2b(onto, tomasys)

    #for testing YUMI in error
    # sys_state = SystemState(yumi_status = 1, camera_status = 1, tag_detection_status = 99) # no tag detected
    # sys_state = SystemState(yumi_status = 1, camera_status = 99, tag_detection_status = 1) # camera error

    timer = rospy.Timer(rospy.Duration(3.), timer_cb)

    graph_manipulation_client = actionlib.SimpleActionClient(
            'cheops_graph_manipulation_action_server',
            GraphManipulationActionAction)
    graph_manipulation_client.wait_for_server()
    rospy.spin()
