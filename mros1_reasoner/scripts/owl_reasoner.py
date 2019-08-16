from owlready2 import *

"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manipulate the ontology

README
- I created a python vrit env for this: $ pew new --python=python3 owlready2
- installed owlready2 in my ws: $ pip install Owlready2
- Make sure you are working in the Virtual environment: $ pew workon owlready2
- run this script by: $ python owl_reasoner.py

"""

'''
     * REASONING best available FD for objective_ins in error
     * @param o the objective in error
     * @return besdt_fd the best FD available (fd_efficacy is the criteria)
'''

test = 3

def obtainBestFunctionDesign(o):
    f = o.typeF
    # get fds for Function F
    fds = []
    for fd in list(tomasys.FunctionDesign.instances()):
        if fd.solves == f:
            fds.append(fd)
    print("\nFunctionDesigns available for obj: ",fds)
    aux = 0
    best_fd = None
    for fd in fds:
        # FILTER if FD realisability is NOT FALSE (TODO check SWRL rules are complete for this)
        print("FD ", fd, fd.fd_realisability)
        if fd.fd_realisability != False:
            # FILTER if the FD error log does NOT contain the current objective
            print("Fd: ", fd, "error_log: ", fd.fd_error_log)
            if not o in fd.fd_error_log:
                if fd.fd_efficacy > aux:
                    best_fd = fd
                    aux = fd.fd_efficacy
    if ( best_fd == None ):
        print("*** OPERATOR NEEDED, NO SOLUTION FOUND ***")
        return None
    else:
        print("\nBest FD available", best_fd)
        return best_fd


'''
To solve the given Objective, recursively Grounds the required hierarchy of
sub-Objectives and Function Groundings
'''
def groundObjective(o, cspecs):
    print("=> Reasoner grounding objective: ", o)
    fd = obtainBestFunctionDesign(o)
    if( fd == None ):
        print("*** Objective ", o,"cannot be realised ***")
        return
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

# Load tomasys and abb_scenario2 ontologies
onto_path.append("../../../mc_mdl_tomasys/") # local folder to search for ontologies
onto_path.append("../../../mc_mdl_abb/") # local folder to search for ontologies

tomasys = get_ontology("tomasys.owl").load()
onto = get_ontology("abb_scenario2.owl").load()

# TODO: replace using ROS instrospection

if test == 1:
    print("\nRunning TEST1: objective to detect tag in ERROR\n")
    # Initial deployment
    fd = onto.search(iri = "*fd_detect_tag_poses_1")[0]
    f = onto.search(iri = "*f_detect_tag_poses")[0]
    camera = onto.search(iri = "*c_camera")[0]
    tag_detector = onto.search(iri = "*#c_tag_detector")[0]
    role_tag_detector = onto.search(iri = "*r_tag_detector_fd_detect_tag_poses_1")[0]
    role_camera = onto.search(iri = "*r_camera_fd_detect_tag_poses_1")[0]

    fg = tomasys.FunctionGrounding("fg_detect_tag_1", namespace = onto)
    o = tomasys.Objective("o_detect_ws1_tag", namespace = onto)
    b1 = tomasys.Binding(namespace = onto)
    b2 = tomasys.Binding(namespace = onto)
    b1.binding_component = camera
    b1.binding_role = role_camera
    b2.binding_component = tag_detector
    b2.binding_role = role_tag_detector
    o.typeF = f
    fg.realises = o
    fg.typeFD = fd
    fg.hasBindings.append(b1)
    fg.hasBindings.append(b2)

    # UPDATE components statuses
    print("\nTest1 objective in error: ", o)
    o.o_status = False
    onto.save(file = "tmp.owl", format = "rdfxml")

elif test == 2:
    print("\nRunning TEST2: camera in ERROR\n")

    # Initial deployment
    fd = onto.search(iri = "*fd_detect_tag_poses_1")[0]
    f = onto.search(iri = "*f_detect_tag_poses")[0]
    camera = onto.search(iri = "*#c_camera")[0]
    tag_detector = onto.search(iri = "*#c_tag_detector")[0]
    role_tag_detector = onto.search(iri = "*r_tag_detector_fd_detect_tag_poses_1")[0]
    role_camera = onto.search(iri = "*r_camera_fd_detect_tag_poses_1")[0]

    fg = tomasys.FunctionGrounding("fg_detect_tag_1", namespace = onto)
    o = tomasys.Objective("o_detect_ws1_tag", namespace = onto)
    b1 = tomasys.Binding(namespace = onto)
    b2 = tomasys.Binding(namespace = onto)
    b1.binding_component = camera
    b1.binding_role = role_camera
    b2.binding_component = tag_detector
    b2.binding_role = role_tag_detector
    o.typeF = f
    fg.realises = o
    fg.typeFD = fd
    fg.hasBindings.append(b1)
    fg.hasBindings.append(b2)

    # UPDATE components statuses
    for i in list(tomasys.ComponentState.instances()) :
        print(i)
        if i.name == "c_camera":
            i.c_status = False
        else:
            i.c_status = True
        print(i, " c_status= ", i.c_status)
    onto.save(file = "tmp.owl", format = "rdfxml")


elif test == 3:
    print("\nRunning TEST3: yumi 2 arms in ERROR\n")

    # Initial deployment
    fd = onto.search(iri = "*fd_build_2arms")[0]
    f = onto.search(iri = "*f_build_pyramid")[0]
    yumi = onto.search(iri = "*#c_yumi")[0]
    role_yumi2 = onto.search(iri = "*r_yumi2a")[0]

    fg = tomasys.FunctionGrounding("fg_build2", namespace = onto)
    o = tomasys.Objective("o_build_pyramid", namespace = onto)
    b = tomasys.Binding(namespace = onto)
    b.binding_component = yumi
    b.binding_role = role_yumi2
    o.typeF = f
    fg.realises = o
    fg.typeFD = fd
    fg.hasBindings.append(b)

    # UPDATE components statuses
    for i in list(tomasys.ComponentState.instances()) :
        if i.name == "c_yumi":
            i.c_status = False
        else:
            i.c_status = True
        print(i, " c_status= ", i.c_status)
    onto.save(file = "tmp.owl", format = "rdfxml")

else:
     print("\nRunning default metacontrol reasoning using monitoring input")


# world closure
# - FDs are all realisable unless negated later - TODO this causes ontology inconsistency, probably we hsould remove
# for fd in list(tomasys.FunctionDesign.instances()) :
#     fd.fd_realisability = True

# REASON objective(s) in error
sync_reasoner_pellet(infer_property_values = True, infer_data_property_values = True)
# Not working yet
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
