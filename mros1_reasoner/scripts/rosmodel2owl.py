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

from owlready2 import *

onto_file ='ros_navigation.owl'
onto = None
# Load ontologies: application model and tomasys
def loadOntology(file):
    onto_path.append(rospack.get_path('mc_mdl_tomasys')+'/') # local folder to search for ontologies
    onto_path.append(rospack.get_path('mc_mdl_abb')+'/') # local folder to search for ontologies
    global tomasys, onto
    tomasys = get_ontology("tomasys.owl").load()  # TODO initilize tomasys using the import in the application ontology file
    onto = get_ontology(file).load()


if __name__ == '__main__':
    loadOntology(onto_file)
    rospy.loginfo("Loaded domain ontology: " + onto_file)


    # code to load RosModel and parse it

    # for each RosSystem:
    # - create a FunctionDesign
    fd = tomasys.FunctionDesign("name_of_RosSystem", namespace=onto)
    # - create a QualityAttribute expected value for the FunctionDesign with the type indicated by the name of the param in the RosSystem and the value of the param (e.g. 0.5)
    # example:
    qa = tomasys.QAValue("name_of_RosSystem_param_name",
                         namespace=onto, isQAtype=onto.search_one(iri="*energy"), hasValue=0.5)
    fd.hasQAEstimation = [qa]
    # END example

    # save the ontology to a file
    onto.save(file="rosmodel.owl", format="rdfxml")

