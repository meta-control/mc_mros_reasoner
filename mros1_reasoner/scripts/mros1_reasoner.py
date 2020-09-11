#!/usr/bin/env python
"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manipulate the ontology

README
- I created a python vrit env for this: $ pew new --python=python3 owlready2
- installed owlready2 in my ws: $ pip install Owlready2
- Make sure you are working in the Virtual environment: $ pew workon owlready2
- run this script by: $ python owl_reasoner.py

"""
import rospy
import rospkg
from owlready2 import *

import actionlib

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from metacontrol_msgs.msg import MvpReconfigurationAction, MvpReconfigurationGoal, \
                                GraphManipulationActionAction,  GraphManipulationActionGoal, \
                                GraphManipulationMessage, SystemState

from collections import defaultdict

import argparse
from decimal import Decimal

from threading import Lock

import signal, sys

from tomasys import *

# For debugging purposes: saves state of the KB in an ontology file
# TODO move to library
# TODO save file in a temp location
def save_ontology_exit(signal, frame):
    onto.save(file="error.owl", format="rdfxml")
    sys.exit(0)

signal.signal(signal.SIGINT, save_ontology_exit)

# Initialize global variables
tomasys = None    # owl model with the tomasys ontology
onto = None       # owl model with the application model as individuals of tomasys classes
mock = True       # whether we are running a mock system (True), so reasoning happens in isolation, or connected to the real system
grounded_configuration = None   # name of the current system configuration, as stored in KB

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# Lock to ensure safety of tQAvalues
lock = Lock()

# NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
# Initializes the KB according to 2 cases:
# - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
# - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
def initKB(onto, tomasys, config_name):
    
    rospy.loginfo('KB initialization:\n \t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety \n \t - If an Objective instance is not found in the owl file, a default o_navigate is created.' )

    #Root objectives
    objectives = onto.search(type=tomasys.Objective)
    # if no objectives in the OWL file, standard navigation objective is assumed
    if objectives == []:
        rospy.loginfo('Creating default Objective o_navigateA with default NFRs')
        o = tomasys.Objective("o_navigateA", namespace=onto,
                            typeF=onto.search_one(iri="*f_navigate"))

        # Read NFRs from rosparam
        if not rospy.has_param('/nfr_energy'):
            rospy.logwarn(
                'No value in rosparam server for /nfr_energy, setting it to 0.5')
            rospy.set_param('/nfr_energy', 0.5)
        else:
            nfr_energy_value = float(rospy.get_param('/nfr_energy'))
        
        if not rospy.has_param('/nfr_safety'):
            rospy.logwarn(
                'No value in rosparam server for /nfr_energy, setting it to 0.8')
            rospy.set_param('/nfr_safety', 0.8)
        else:
            nfr_safety_value = float(rospy.get_param('/nfr_safety'))

        # Load NFRs in the KB
        nfr_energy = tomasys.QAvalue("nfr_energy", namespace=onto, isQAtype=onto.search_one(
            iri="*energy"), hasValue=nfr_energy_value)
        nfr_safety = tomasys.QAvalue("nfr_safety", namespace=onto, isQAtype=onto.search_one(
            iri="*safety"), hasValue=nfr_safety_value)
        
        # Link NFRs to objective
        o.hasNFR.append(nfr_energy)
        o.hasNFR.append(nfr_safety)

        # # Function Groundings and Objectives
        fg = tomasys.FunctionGrounding("fg_{}".format(config_name), namespace=onto, typeFD=onto.search_one(iri="*{}".format(config_name)), solvesO=o)
      
    elif len(objectives) == 1:
        o = objectives[0]
        fg = tomasys.FunctionGrounding("fg_" + o.name.replace('o_',''), namespace=onto, typeFD=obtainBestFunctionDesign(o, tomasys), solvesO=o)
        rospy.logwarn('Objective, NFRs and initial FG are generated from the OWL file')
    else:
        rospy.logerr('Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

    # For debugging InConsistent ontology errors, save the ontology before reasoning
    onto.save(file="tmp_debug.owl", format="rdfxml")


# MVP: callback for diagnostic msg received from QA Observer
def callbackDiagnostics(msg):
    global onto
    for diagnostic_status in msg.status:
        # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
        if diagnostic_status.message == "binding error":
            updateBinding(diagnostic_status)
        if diagnostic_status.message == "QA status":
            updateQA(diagnostic_status)

# the DiagnosticStatus message process contains, per field
# - message: "binding_error"
# - name: name of the fg reported, as named in the OWL file
# - level: values 0 and 1 are mapped to nothing, values 2 or 3 are mapper to fg.status="INTERNAL_ERROR"
def updateBinding(diagnostic_status):
    rospy.loginfo("binding error received")
    fg = onto.search_one(iri="*{}".format(diagnostic_status.name))
    if fg == None:
        rospy.logwarn("Unkown Function Grounding: %s", diagnostic_status.name)
        return
    if diagnostic_status.level > 1:
        fg.fg_status = "INTERNAL_ERROR"
    else:
        rospy.logwarn("Diagnostics message received for %s with level %d, nothing done about it." % (fg.name, diagnostic_status.level))


# To reset the inferences that no longer hold due to adaptation
def resetOntologyStatuses():
    for o in list(tomasys.Objective.instances()):
        o.o_status = None

# update QA value based on incoming diagnostic
def updateQA(diagnostic_status):
    rospy.logwarn("QA value received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))

    # Find the FG with the same name that the one in the QA message (in diagnostic_status.name)
    fg = next((fg for fg in tomasys.FunctionGrounding.instances() if fg.name == diagnostic_status.name), None)
    if fg == None:
        rospy.logwarn("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
        fg = tomasys.FunctionGrounding.instances()[0]
    qa_type = onto.search_one(iri="*{}".format(diagnostic_status.values[0].key))
    if qa_type != None:
        value = float(diagnostic_status.values[0].value)
        rospy.loginfo("QA value received!\tTYPE: {0}\tVALUE: {1}".format(qa_type.name, value))
        with lock:
            updateQAvalue(fg, qa_type, value)
    else:
        rospy.logwarn("Unsupported QA TYPE received: %s ", str(diagnostic_status.values[0].key))

## main metacontrol loop
def timer_cb(event):
    global onto
    global grounded_configuration
    rospy.loginfo('Entered timer_cb for metacontrol reasoning')

    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    rospy.loginfo('  >> Started MAPE-K ** Analysis (ontological reasoning) **')
    with lock:
        try:
            with onto:
                sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
        except owlready2.base.OwlReadyInconsistentOntologyError as err:
            rospy.logerr("Reasoning error: %s", str(err))
            onto.save(file="error.owl", format="rdfxml")
    rospy.loginfo('     >> Finished ontological reasoning)')

    # PRINT system status
    print_ontology_status(tomasys)

    # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
    objectives_internal_error = evaluateObjectives(tomasys)
    if not objectives_internal_error:
        rospy.loginfo("No Objectives in status ERROR: no adaptation is needed")
    else:
        rospy.logwarn("Objectives in status ERROR: {}".format([o.name for o in objectives_internal_error]) )
    rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')

    # ADAPT MAPE -Plan & Execute
    rospy.loginfo('  >> Started MAPE-K ** PLAN adaptation **')
    # CHEOPS - TODO TEST
    if len(objectives_internal_error) > 1:
        # CHEOPS Ground a solution hierarchy for each root objective in error. We assume here that root_objectives do not share intermediate objectives
        cspecs = []
        for o in objectives_internal_error:
            groundObjective(o, cspecs)

        str_specs = [cs.name for cs in cspecs]
        rospy.logerr("RESULT CONFIG: {}".format(str_specs) ) # for DEBUGGING in csv
        if len(str_specs) != 0:
            request_reconfiguration()  # CHEOPS request reconfiguration by sending cspecs names

    elif len(objectives_internal_error) == 1 :
        o = objectives_internal_error[0]
        rospy.loginfo("=> Reasoner searches FD for objective: {}".format(o.name) )
        fd = obtainBestFunctionDesign(o, tomasys)
        rospy.loginfo('  >> Finished MAPE-K ** Plan adaptation **')
        # MVP to request new configuration
        if fd != ["safe_shutdown"]:
            rospy.loginfo('  >> Started MAPE-K ** EXECUTION **')
            result = request_configuration(fd)
            rospy.loginfo('  >> Finished MAPE-K ** EXECUTION **')
            # Adaptation feedback:
            if result == 1: # reconfiguration executed ok
                rospy.logwarn("= RECONFIGURATION SUCCEEDED =") # for DEBUGGING in csv         
                # update the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
                fg = onto.search_one(
                    solvesO=objectives_internal_error[0])
                destroy_entity(fg)
                fg = tomasys.FunctionGrounding(
                    "fg_"+fd.name.replace('fd_', ''), namespace=onto, typeFD=fd, solvesO=objectives_internal_error[0])
                ## Set new grounded_configuration
                grounded_configuration = str(fg.name)

                resetKBstatuses(tomasys)

            elif result == -1:
                rospy.logerr("= RECONFIGURATION UNKNOWN =") # for DEBUGGING in csv
            else:
                rospy.logerr("= RECONFIGURATION FAILED =") # for DEBUGGING in csv

        else:
            rospy.logerr(
                "No FD found to solve Objective {}, requesting shutdown not available".format(o.name)) # for DEBUGGING in csv

    else:
        rospy.loginfo("- NO ADAPTATION NEEDED -")

    rospy.loginfo('Exited timer_cb for metacontrol reasoning')


# for MVP with QAs - request the FD.name to reconfigure to
def request_configuration(fd):
    rospy.logwarn_throttle(1., 'New Configuration requested: {}'.format(fd.name))

    goal = MvpReconfigurationGoal()
    goal.desired_configuration_name = fd.name
    rosgraph_manipulator_client.send_goal(goal)
    rosgraph_manipulator_client.wait_for_result()
    result = rosgraph_manipulator_client.get_result().result
    rospy.loginfo('Result: {}'.format(result) )
    return result


if __name__ == '__main__':
    # Start rosnode
    rospy.init_node('mros1_reasoner')

    # load ontology
    onto_file = rospy.get_param('/onto_file')
    tomasys, onto = loadTomasysKB(rospack.get_path('mc_mdl_tomasys')+'/tomasys.owl', onto_file)
    rospy.loginfo("Loaded ontology: %s", str(onto_file))

    # initialize the system grounded_configuration (FG or grounded FD) from rosparam
    try:
        grounded_configuration = rospy.get_param('/desired_configuration')
        rospy.loginfo('grounded_configuration initialized to: %s', grounded_configuration)

    except KeyError:
        grounded_configuration = None
        rospy.logwarn('grounded_configuration not found in the param server')

    # initialize KB with the ontology
    initKB(onto, tomasys, grounded_configuration)

    #Start interfaces
    sub_diagnostics = rospy.Subscriber('/diagnostics', DiagnosticArray, callbackDiagnostics)

    timer = rospy.Timer(rospy.Duration(2.), timer_cb)

    rosgraph_manipulator_client = actionlib.SimpleActionClient(
        'rosgraph_manipulator_action_server',
        MvpReconfigurationAction)
    # rosgraph_manipulator_client.wait_for_server()

    rospy.spin()
