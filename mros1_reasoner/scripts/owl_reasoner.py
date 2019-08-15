from owlready2 import *

"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manipulate the ontology

README
- I created a python vrit env for this: $ pew new --python=python3 owlready2
- installed owlready2 in my ws: $ pip install Owlready2
- run this script by: $ python owlready_mros_reasoner.py

"""

'''
     * REASONING best available FD for objective_ins in error
     * @param o the objective in error
     * @return besdt_fd the best FD available (fd_efficacy is the criteria)
'''

test = 1

def obtainBestFunctionDesign(o):
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tomasys.FunctionDesign.instances()):
        if fd.solves == f:
            fds.append(fd)
    print("\nFunctionDesigns available for obj: ",fds)
    aux = 0
    for fd in fds:
        # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
        if fd.fd_realisability:
            # FILTER if the FD error log does NOT contain the current objective
            if not o in fd.fd_error_log:
                if fd.fd_efficacy > aux:
                    best_fd = fd
                    aux = fd.fd_efficacy
    print("\nBest FD available", best_fd)
    return best_fd


'''
To solve the given Objective, recursively Grounds the required hierarchy of
sub-Objectives and Function Groundings
'''
def groundObjective(o, cspecs):
    fd = obtainBestFunctionDesign(o)
    fg = tomasys.FunctionGrounding("fg_")

    for r in fd.roles:
        b = tomasys.Binding("b_")
        b.binding_role = r
        fg.has_bindings = b # TODO WORKING HERE
        cspecs.append(r.roleDef)

    for f in fd.requires:
        ob = tomasys.Objective("o_")
        ob.typeF = f
        fg.needs.append(ob)
        groundObjective(ob, cspecs)

    return cspecs

# Load tomasys and abb_scenario2 ontologies
onto_path.append("../../../mc_mdl_tomasys/") # local folder to search for ontologies
onto_path.append("../../../mc_mdl_abb/") # local folder to search for ontologies

tomasys = get_ontology("tomasys.owl").load()
onto = get_ontology("abb_scenario2.owl").load()

# UPDATE components statuses
# TODO: replace using ROS instrospection

if test == 1:
    print("\nTest1 objective in error:")
    for o in onto.search(iri = "*o_build_pyramid"):
            o.o_status = False

elif test == 2:
    for i in list(tomasys.ComponentState.instances()) :
        if i.name == 'abb_scenario2.c_camera':
            i.c_status = False
        else:
            i.c_status = True
        print(i, " c_status= ", i.c_status)

elif test == 3:
    for i in list(tomasys.ComponentState.instances()) :
        if i.name == 'abb_scenario2.c_yumi':
            i.c_status = False
        else:
            i.c_status = True
        print(i, " c_status= ", i.c_status)
else:
     print("\nRunning default metacontrol reasoning using monitoring input")


# world closure
# - FDs are all realisable unless negated later - TODO does not seem to be the case!!
for fd in list(tomasys.FunctionDesign.instances()) :
    fd.fd_realisability = True

# REASON objective(s) in error
# sync_reasoner()
sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)

# PRINT system status
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

# RECONFIGURATION REASONING
# compute the best Function Design possible to address objectives in false status (aka in ERROR)
# TODO: filter out objectives no longer needed in the hierarchy

# init objectives in error
objectives_in_error = []
for o in list(tomasys.Objective.instances() ):
    if not o.o_status == True:
        objectives_in_error.append(o)
print("\nObjectives in error:", objectives_in_error)

# Ground a solution hierarchy for each root objective in error. We assume here that root_objectives do not share intermediate objectives
cspecs = []
for o in objectives_in_error:
    groundObjective(o, cspecs)

print("\nComponent specifications: ", cspecs)
