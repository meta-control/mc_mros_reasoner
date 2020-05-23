#!/usr/bin/env python
'''
authors: c.h.corbato@tudelft.nl

This script generates an .owl file from a RosModel
It uses Owlready2 to manipulate the OWL ontology

INPUT:
- RosModel: not sure if a system.rossystem and/or multiple .rossystem files with the configurations of navigation 
- tomasys.owl: contains the tomasys metamodel with the ontology classes
- onto_file (default value=ros_navigation.owl): contains individuals and rules for the domain (e.g. ROS navihation stack)
'''
import rospy
import rospkg
from rosgraph_monitor.parser import ModelParser
from pyparsing import ParseResults

from owlready2 import *
from os import listdir

meta_pack = "metacontrol_move_base_configurations"
onto_file ='ros_navigation.owl'

r = rospkg.RosPack()


def get_all_files():
    file_list = []
    config_pkgs = r.get_depends(meta_pack, implicit=False)
    for pkg in config_pkgs:
        path = r.get_path(pkg)
        for f in listdir(path):
            if not f.endswith(".rossystem"):
                continue
            file_list.append(path + "/" + f)
    return file_list

onto = None
# Load ontologies: application model and tomasys
def loadOntology(file):
    onto_path.append(r.get_path('mc_mdl_tomasys')+'/') # local folder to search for ontologies
    onto_path.append(r.get_path('mc_mdl_abb')+'/') # local folder to search for ontologies
    global tomasys, onto
    tomasys = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file
    onto = get_ontology(file).load()

import os
if __name__ == '__main__':
    rossys_files = get_all_files()
    # print(rossys_files)

    loadOntology(onto_file)
    rospy.loginfo("Loaded domain ontology: " + onto_file)


    # code to load RosModel and parse it
    for file in rossys_files:
        parser = ModelParser(file)
        # print(parser.parse().dump())

        model = parser.parse()
        sys_name = model.system_name[0]
        safety_attr = model.global_parameters[0]
        energy_attr = model.global_parameters[1]

        # for each RosSystem:
        # - create a FunctionDesign
        fd = tomasys.FunctionDesign(sys_name, namespace=onto)
        # - create a QualityAttribute expected value for the FunctionDesign with the type indicated by the name of the param in the RosSystem and the value of the param (e.g. 0.5)
        # example:
        qa = tomasys.QAvalue(energy_attr.param_name[0],
                            namespace=onto, isQAtype=onto.search_one(iri="*energy"), hasValue=energy_attr.param_value[0])
        fd.hasQAestimation.append(qa)
        # END example

        # save the ontology to a file
        onto.save(file="rosmodel.owl", format="rdfxml")

