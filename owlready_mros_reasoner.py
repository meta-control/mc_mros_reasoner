from owlready2 import *

"""
author: c.h.corbato@tudelft.nl
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
sync_reasoner()
for i in onto.Objective.instances() :
    print(i)
    if i.o_status == False:
        print(i, " o_status= ", i.o_status)
