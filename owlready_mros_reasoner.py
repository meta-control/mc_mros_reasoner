from owlready2 import *

"""
author: c.h.corbato@tudelft.nl
using Owlready to manipulate the ontology
Problem: not possible to access inferred facts in individuals, because it is not supported by Owlready yet
"""

# Load unexmin metacontrol ontology
onto = get_ontology("file:///home/chcorbato/mros_ws/tomasys/metacontrol_unexmin.owl")
onto.load()

# UPDATE components statuses
# TODO: replace using ROS instrocpection
for i in onto.ComponentState.instances() :
    if i == onto.motor4:
        i.c_status = False
    else:
        i.c_status = True
    print(i, " c_status= ", i.c_status)

# REASON objective(s) in error
# sync_reasoner()
sync_reasoner_pellet()
for i in onto.Objective.instances() :
    print(i)
    # the following does not work because the inferred facts in individuals (property values) cannot be accessed
    if i.o_status == False:
        print(i, " o_status= ", i.o_status)
