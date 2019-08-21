'''
Initiatize the reasoner Kknowledge base: load ontology and asserts initial state
'''
def init_abb_2a(onto, tomasys):
    # Initial system state
    fd = onto.search_one(iri = "*fd_build_2arms")
    f = onto.search_one(iri = "*f_build_pyramid")
    yumi = onto.search_one(iri = "*#c_yumi")[0]
    role_yumi2 = onto.search_one(iri = "*r_yumi2a")

    fg = tomasys.FunctionGrounding("fg_build2", namespace = onto)
    o = tomasys.Objective("o_build_pyramid", namespace = onto)
    b = tomasys.Binding(namespace = onto)
    b.binding_component = yumi
    b.binding_role = role_yumi2
    o.typeF = f
    fg.realises = o
    fg.typeFD = fd
    fg.hasBindings.append(b)


def init_abb_2b(onto, tomasys):
    # Initial system state
    yumi = onto.search_one(iri = "*#c_yumi")
    camera = onto.search_one(iri = "*#c_camera")
    tag_detector = onto.search_one(iri = "*#c_tag_detector")

    #Root objectives
    o = tomasys.Objective("o_build_pyramid", namespace = onto, typeF=onto.search_one(iri = "*f_build_pyramid") )

    # Function Groundings
    fg = tomasys.FunctionGrounding("fg_build2", namespace = onto, typeFD= onto.search_one(iri = "*fd_build_2arms"), realises=o,
        hasBindings=[tomasys.Binding(namespace = onto, binding_component = yumi, binding_role=onto.search_one(iri = "*r_yumi2a"))])
