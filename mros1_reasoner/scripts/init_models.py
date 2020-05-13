'''
Different methods to initiatize the reasoner Knowledge base: load ontology and asserts initial state
'''
def init_abb_2a(onto, tomasys): # use with abb_scenario2.owl model
    # Initial system state
    yumi = onto.search_one(iri = "*#c_yumi")

    # Root objectives
    o = tomasys.Objective("o_build_pyramid", namespace = onto, typeF=onto.search_one(iri = "*f_build_pyramid"))

    fg = tomasys.FunctionGrounding("fg_build2", namespace = onto, typeFD=onto.search_one(iri = "*fd_build_2arms"),
            realises = o,
            hasBindings = [tomasys.Binding(namespace = onto, binding_component = yumi, binding_role = onto.search_one(iri = "*r_yumi2a"))])


def init_abb_2b(onto, tomasys): # use with abb_dualarm_mm_complete.owl model
    # Initial system state
    yumi = onto.search_one(iri = "*#c_yumi")
    camera = onto.search_one(iri = "*#c_camera")
    tag_detector = onto.search_one(iri = "*#c_tag_detector")
    # tag_locator = onto.search_one(iri = "*#") #TODO create, not search

    #Root objectives
    o = tomasys.Objective("o_build_pyramid", namespace = onto, typeF=onto.search_one(iri = "*f_build_pyramid") )

    # Function Groundings and Objectives
    o2 = tomasys.Objective("o_tag_localization", namespace = onto, typeF=onto.search_one(iri = "*f_locate_robot_tag") )
    fg = tomasys.FunctionGrounding("fg_build_2arms", namespace = onto, typeFD= onto.search_one(iri = "*fd_build_2arms"),
            solvesO = o,
            needs = [o2],
            hasBindings = [tomasys.Binding(namespace = onto, binding_component = yumi, binding_role = onto.search_one(iri = "*r_yumi2a"))])
    o3 = tomasys.Objective("o_detect_tag", namespace = onto, typeF=onto.search_one(iri = "*f_detect_tag_poses") )
    fg = tomasys.FunctionGrounding("fg_locate_tag", namespace = onto, typeFD= onto.search_one(iri = "*fd_locate_ws1_tag"),
            solvesO = o2,
            needs = [o3],
            hasBindings = [tomasys.Binding(namespace = onto, binding_component = None, binding_role = onto.search_one(iri = "*r_tag_calibration_node"))])


    fg = tomasys.FunctionGrounding("fg_detect_tag", namespace = onto, typeFD= onto.search_one(iri = "*fd_detect_tag_poses_1"),
            solvesO = o3,
            hasBindings = [tomasys.Binding(namespace = onto, binding_component = camera, binding_role = onto.search_one(iri = "*r_camera_fd_detect_tag_poses_1")),
                           tomasys.Binding(namespace = onto, binding_component = tag_detector, binding_role = onto.search_one(iri = "*r_tag_detector_fd_detect_tag_poses_1"))
                            ])


def init_mvp(onto, tomasys):  # use with mvp.owl model
    # NFRs on QAs
    nfr_energy = tomasys.QAvalue("nfr_energy", namespace=onto, isQAtype=onto.search_one(
        iri="*energy"), hasValue=1.5)

    # Initial system state

    #Root objectives
    o = tomasys.Objective("o_navigateA", namespace=onto,
                          typeF=onto.search_one(iri="*f_navigate"), hasNFR=[nfr_energy])

    # # Function Groundings and Objectives
    fg = tomasys.FunctionGrounding("fg_nav_fast", namespace=onto, typeFD=onto.search_one(
        iri="*fd_navigate_fast"), solvesO=o)

    print('save ontology')
    # For debugging InConsistent ontology errors, save the ontology before reasoning
    onto.save(file="tmp_debug.owl", format="rdfxml")


def init_abb_3(onto, tomasys): # use with abb_scenario3.owl model
    # Initial system state

    #Root objectives
    o = tomasys.Objective("o_navigateA", namespace = onto, typeF=onto.search_one(iri = "*f_navigate") )
    o.o_nfr_energy = 1.4

    # # Function Groundings and Objectives
    fg = tomasys.FunctionGrounding("fg_nav_fast", namespace = onto, typeFD= onto.search_one(iri = "*fd_navigate_fast"), solvesO=o)

    print('save ontology')
    onto.save(file = "tmp_debug.owl", format = "rdfxml") # For debugging InConsistent ontology errors, save the ontology before reasoning
