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

import csv

# File that contains the Domain model ontology
onto_file ='ros_navigation.owl' # TODO: define as param which domain ontology to load

ros_root = rospkg.get_ros_root()
r = rospkg.RosPack()

onto = None
# Load ontologies: application model and tomasys
def loadOntology(file):
    onto_path.append(r.get_path('mc_mdl_tomasys')+'/') # local folder to search for ontologies
    onto_path.append(r.get_path('mc_mdl_abb')+'/') # local folder to search for ontologies
    global tomasys, onto
    tomasys = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file
    onto = get_ontology(file).load()

import os, sys
if __name__ == '__main__':
    loadOntology(onto_file)
    rospy.loginfo("Loaded domain ontology: " + onto_file)

    # TODO: hardcoded here that all FDs from .rossystem files solve F navigate
    f_navigate = onto.search_one(iri="*f_navigate")

    if f_navigate == None:
        print("The domain ontology provided does not contain a Function f_navigate")
        sys.exit(0)

    # code to load RosModel and parse it
    # my_path = os.path.abspath(os.path.dirname(__file__))
    # base_path = os.path.join(my_path, "../MoveBaseConfigurations/")
    base_path = os.path.abspath("/home/chcorbato/rosin_paper_ws/src/metacontrol_move_base_configurations")
    
    # open the csv file with the expected/estimated values for the QA of the FDs
    with open('../expected_qas.csv', mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for i in [1, 2, 3]:
            for j in [1, 2, 3]:
                for k in [1, 2, 3]:
                    config_name = "f{0}_v{1}_r{2}".format(i, j, k)
                    file_name = config_name + ".rossystem"
                    file_path = os.path.join(base_path, config_name, file_name)
                    parser = ModelParser(file_path)
                    # print(parser.parse().dump())
                    model = parser.parse()
                    sys_name = model.system_name[0]
                    safety_attr = model.global_parameters[0]
                    energy_attr = model.global_parameters[1]
                    # print(sys_name)
                    # print(safety_attr.param_name[0])
                    # print(energy_attr.param_name[0])
                    # print(safety_attr.param_value[0])
                    # print(energy_attr.param_value[0])

                    # create a FunctionDesign
                    fd = tomasys.FunctionDesign(sys_name, namespace=onto, solvesF=f_navigate)
                    
                    # create a QualityAttribute expected value for the FunctionDesign with the type indicated by the name of the param in the RosSystem and the value of the param (e.g. 0.5)
                    # value is read from CSV file instead of rossystem file
                    row = next(csv_reader)  # get next row in CSV                    
                    for qa_param in model.global_parameters:
                        qa_string = qa_param.param_name[0].replace('qa_', '')
                        value = float(row[qa_string])
                        qa = tomasys.QAvalue("{0}_{1}".format(qa_param.param_name[0], sys_name), namespace=onto, isQAtype=onto.search_one(iri="*"+qa_string), hasValue=value)
                        fd.hasQAestimation.append(qa)
                    
                    # add QA performance from csv even if not in rossystem model.global_parameters
                    value = float(row['performance'])
                    qa = tomasys.QAvalue("{0}_{1}".format('qa_performance', sys_name),     namespace=onto, isQAtype=onto.search_one(iri="*performance"), hasValue=value)
                    fd.hasQAestimation.append(qa)

    # save the ontology to a file
    onto.save(file="kb.owl", format="rdfxml")

