from owlready2 import *

"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manipulate the ontology

README
- I created a python vrit env for this: $ pew new --python=python3 owlready2
- installed owlready2 in my ws: $ pip install Owlready2
- run this script by: $ python owlready_mros_reasoner.py

"""

# Load unexmin metacontrol ontology
onto_path.append("/home/chcorbato/mros_ws/tomasys/") # local folder to search for ontologies
tomasys = get_ontology("file:///home/chcorbato/mros_ws/tomasys/tomasys.owl").load()
onto = get_ontology("file:///home/chcorbato/mros_ws/tomasys/abb_scenario1.owl").load()

# UPDATE components statuses
# TODO: replace using ROS instrocpection
for i in list(tomasys.ComponentState.instances()) :
    if i.iri == 'http://abb_scenario1#c_camera':
        i.c_status = False
    else:
        i.c_status = True
    print(i, " c_status= ", i.c_status)

# REASON objective(s) in error
# sync_reasoner()
sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)

print("\nComponents Statuses:")
for i in list(tomasys.ComponentState.instances()) :
    print(i, i.c_status)

print("\nBindings Statuses:")
for i in list(tomasys.Binding.instances()) :
    print(i, i.b_status)

print("\nFG Statuses:")
for i in list(tomasys.FunctionGrounding.instances()) :
    print(i, i.fg_status)

print("\nObjectives Statuses:")
for i in list(tomasys.Objective.instances()) :
    print(i, i.o_status)

print("\nCC availability:")
for i in list(tomasys.ComponentClass.instances()) :
    print(i, i.cc_availability)


print("\nFD realisability:")
for i in list(tomasys.FunctionDesign.instances()) :
    print(i, i.fd_realisability)
