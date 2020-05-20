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

from init_models import *

# Initialize global variables
tomasys = None    # owl model with the tomasys ontology
onto = None       # owl model with the application model as individuals of tomasys classes
mock = True       # whether we are running a mock system (True), so reasoning happens in isolation, or connected t the real system

# get an instance of RosPack with the default search paths
rospack = rospkg.RosPack()

# Load ontologies: application model and tomasys
def loadOntology(file):
    onto_path.append(rospack.get_path('mc_mdl_tomasys')+'/') # local folder to search for ontologies
    onto_path.append(rospack.get_path('mc_mdl_abb')+'/') # local folder to search for ontologies
    global tomasys, onto
    tomasys = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file
    onto = get_ontology(file).load()


# MVP metacontrol PLAN: returns the FD that is estimated to maximize QA (TODO trade-off) for a given objective o
def obtainBestFunctionDesign(o):
    global tomasys, onto
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tomasys.FunctionDesign.instances()):
        if fd.solvesF == f:
            fds.append(fd)
    print("== FunctionDesigns available for obj: ", [fd.name for fd in fds])
    print("Objective NFR ENERGY: ", o.hasNFR)
    
    # fiter fds to only those available
    # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
    realisable_fds = [fd for fd in fds if fd.fd_realisability != False]
    print("== FunctionDesigns REALISABLE for obj: ", [fd.name for fd in realisable_fds])
    # discard FDs already grounded for this objective when objective in error
    suitable_fds= [fd for fd in fds if (not o in fd.fd_error_log)]
    print("== FunctionDesigns suitable NOT IN ERROR LOG: ", [fd.name for fd in suitable_fds])
    # discard those FD that will not meet objective NFRs
    fds_for_obj = meetNFRs(o, suitable_fds)
    print("== FunctionDesigns also meeting NFRs: ", [fd.name for fd in fds_for_obj])

    # get best FD based on higher Utility/trade-off of QAs
    if fds_for_obj != []:
        aux = 0
        best_fd = fds_for_obj[0]
        for fd in fds_for_obj:
            u = utility(fd)
            if  u > aux: 
                best_fd = fd
                aux = u
        
        print("> Best FD available", best_fd.name)
        return best_fd
    else:
        print("*** OPERATOR NEEDED, NO SOLUTION FOUND ***")
        return None

def meetNFRs(o, fds):
    filtered = []
    print("== Checking FDs for Objective with NFRs type: ", o.hasNFR[0].isQAtype.name, "and value: ", o.hasNFR[0].hasValue)
    for fd in fds:
        for nfr in o.hasNFR:
            qas = [qa for qa in fd.hasQAestimation if qa.isQAtype==nfr.isQAtype]               
            if qas == []:
                print("\t WARNING FD has no expected value for this QA")
            else:
                if nfr.isQAtype.name == 'energy':
                    if qas[0].hasValue < nfr.hasValue: # specific semantics for energy
                        filtered.append(fd)
                if nfr.isQAtype.name == 'safety':
                    if qas[0].hasValue > nfr.hasValue:  # specific semantics for energy
                        filtered.append(fd)
    if filtered == []:
        print("#### WARNING: no FDs meetf NFRs")

    return filtered

# MVP: compute expected utility based on QA trade-off, the criteria to chose FDs/configurations
def utility(fd):
    # TODO
    return 1



# MVP: select FD to reconfigure to fix Objective in ERROR
def selectFD(o):
    global tomasys, onto
    print("=> Reasoner searches FD for objective: ", o.name)
    fd = obtainBestFunctionDesign(o)
    if(fd == None):
        print("*** Objective ", o.name, "cannot be realised ***")
        return ["safe_shutdown"]
    else:
        return fd


# MVP: callback for diagnostic msg received from QA Observer
def callbackDiagnostics(msg):
    global onto
    for diagnostic_status in msg.status:
        # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
        if diagnostic_status.message == "binding error":
            updateBinding(diagnostic_status)
        if diagnostic_status.message == "QA status":
            updateQA(diagnostic_status)

def updateBinding(msg):
    # TODO handle reporting of a fg.binding in error
    print("binding error received")

# To reset the inferences that no longer hold due to adaptation
def resetOntologyStatuses():
    for o in list(tomasys.Objective.instances()):
        o.o_status = None

# MVP update QA value based on incoming diagnostic
counter=0
def updateQA(diagnostic_status):
    global onto
    global counter
    counter += 1
    #find the FG that solves the Objective with the same name that the one in the QA message
    fg = onto.search_one(solvesO=onto.search_one(iri="*" + "o_navigateA")) #TODO read objective from diagnostic_status
    if fg == None:
        print("ERROR: FG not found")
        return
    print("received QA about: ", fg)
    if diagnostic_status.values[0].key == "energy":
        fg.hasQAvalue.append( tomasys.QAvalue("obs_energy_{}".format(counter), namespace=onto, isQAtype=onto.search_one(
            iri="*energy"), hasValue=float(diagnostic_status.values[0].value)))
    else:
        print('Unsupported QA type different than _energy_') 

def print_ontology_status():
    global onto
    print("\nComponents Statuses:")
    for i in list(tomasys.ComponentState.instances()):
        print(i.name, i.c_status)

    print("\nBindings Statuses:")
    for i in list(tomasys.Binding.instances()):
        print(i.name, i.b_status)

    print("\nFG Statuses:")
    for i in list(tomasys.FunctionGrounding.instances()):
        print(i.name,"\t", i.fg_status, "\t", [ (qa.name, qa.hasValue) for qa in i.hasQAvalue])

    print("\nObjectives Statuses:")
    for i in list(tomasys.Objective.instances()):
        print(i.name, i.o_status, [(nfr.isQAtype, nfr.hasValue) for nfr in i.hasNFR])

    print("\nCC availability:")
    for i in list(tomasys.ComponentClass.instances()):
        print(i.name, i.cc_availability)

    print("\nFDs information:\n NAME \t")
    for i in list(tomasys.FunctionDesign.instances()):
        print(i.name, "\t", i.fd_realisability, "\t", [
              (qa.isQAtype, qa.hasValue) for qa in i.hasQAestimation])



def timer_cb(event):
    global onto
    rospy.loginfo_throttle(1., 'Entered timer_cb for metacontrol reasoning')
    
    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    try:
        sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
    except owlready2.base.OwlReadyInconsistentOntologyError as err:
        print("Reasoning error: {0}".format(err))
        onto.save(file="error.owl", format="rdfxml")

    # PRINT system status
    print_ontology_status()
    
    # EVALUATE and retrieve desired configuration (MAPE - Analysis)
    # init objectives in error
    objectives_internal_error = []
    for o in list(tomasys.Objective.instances() ):
        if o.o_status == "INTERNAL_ERROR":
            objectives_internal_error.append(o)
    print("\nObjectives in error:", [o.name for o in objectives_internal_error] )

    # ADAPT MAPE -Plan & Execute

    # CHEOPS
    if len(objectives_internal_error) > 1:
        # CHEOPS Ground a solution hierarchy for each root objective in error. We assume here that root_objectives do not share intermediate objectives
        cspecs = []
        for o in objectives_internal_error:
            groundObjective(o, cspecs)

        str_specs = [cs.name for cs in cspecs]
        print("RESULT CONFIG: ", str_specs)
        if len(str_specs) != 0:
            request_reconfiguration()  # CHEOPS request reconfiguration by sending cspecs names
    
    # MVP (TODO fix when CHEOPS also only 1)
    elif len(objectives_internal_error) == 1 :
        fd = selectFD(objectives_internal_error[0])
        # MVP to request new configuration
        if fd != ["safe_shutdown"]:
            result = request_configuration(fd)

            # Adaptation feedback: 
            if result == 1: # reconfiguration executed ok
                print("== RECONFIGURATION SUCCEEDED ==")
                # update the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
                fg = onto.search_one(
                    solvesO=objectives_internal_error[0])
                destroy_entity(fg)
                fg = tomasys.FunctionGrounding(
                    "fg_new", namespace=onto, typeFD=fd, solvesO=objectives_internal_error[0])
                resetOntologyStatuses()

            elif result == -1:
                print("== RECONFIGURATION UNKNOWN ==")
            else:
                print("== RECONFIGURATION FAILED ==")

        else:
            print("No FD found to solve Objective, requesting shutdown not available")

    #TODO
    else:
        print("-- NO ADAPTATION NEEDED --")


# for MVP with QAs - request the FD.name to reconfigure to
def request_configuration(fd):
    rospy.logwarn_throttle(1., 'New Configuration requested: {}'.format(fd.name))

    goal = MvpReconfigurationGoal()
    goal.desired_configuration_name = fd.name
    rosgraph_manipulator_client.send_goal(goal)
    rosgraph_manipulator_client.wait_for_result()
    result = rosgraph_manipulator_client.get_result().result
    print('Result: ', result)
    return result


if __name__ == '__main__':
    # load ontology
    onto_file = rospy.get_param('/onto_file')
    loadOntology(onto_file)
    rospy.loginfo("Loaded ontology: " + onto_file)
    # init specific application model using the corresponding init sript
    if onto_file == "abb_scenario2.owl" :
        init_abb_2a(onto, tomasys)
    elif onto_file == "abb_dualarm_mm_complete.owl":
        init_abb_2b(onto, tomasys)
    elif onto_file == "mvp.owl":
        init_mvp(onto, tomasys)
    elif onto_file == "abb_scenario3.owl":
        init_abb_3(onto, tomasys)
    else:
        print("Unknown ontology file: ", onto_file)

    # Start rosnode stuff
    rospy.init_node('mros1_reasoner')
    sub_diagnostics = rospy.Subscriber('/diagnostics', DiagnosticArray, callbackDiagnostics)

    timer = rospy.Timer(rospy.Duration(3.), timer_cb)

    rosgraph_manipulator_client = actionlib.SimpleActionClient(
        'rosgraph_manipulator_action_server',
        MvpReconfigurationAction)
    # rosgraph_manipulator_client.wait_for_server()

    rospy.spin()
