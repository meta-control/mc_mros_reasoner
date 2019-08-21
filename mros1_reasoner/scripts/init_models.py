'''
Initiatize the reasoner Kknowledge base: load ontology and asserts initial state
'''
def init_abb_2a(onto, tomasys):
    # Initial system state
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


def init_abb_2b(onto, tomasys):
    # Initial system state
    yumi = onto.search(iri = "*#c_yumi")[0]
    camera = onto.search(iri = "*#c_camera")[0]
    tag_detector = onto.search(iri = "*#c_tag_detector")[0]

    #Root objectives
    o = tomasys.Objective("o_build_pyramid", namespace = onto, typeF=onto.search(iri = "*f_build_pyramid")[0])

    # role_yumi2 = onto.search(iri = "*r_yumi2a")[0]

    # fg = tomasys.FunctionGrounding("fg_build2", namespace = onto, typeFD= onto.search(iri = "*fd_build_2arms")[0], realises=o)
    # b = tomasys.Binding(namespace = onto)
    # b.binding_component = yumi
    # b.binding_role = role_yumi2
    # o.typeF = f
    # fg.realises = o
    # fg.typeFD = fd
    # fg.hasBindings.append(b)
