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
import roslib

from ros_model_parser.rossystem_parser import RosSystemModelParser
from pyparsing import ParseResults
roslib.load_manifest('rosparam')
import rosparam
from owlready2 import *

ros_root = rospkg.get_ros_root()
rospack = rospkg.RosPack()

onto = None
import os, sys
if __name__ == '__main__':
    # the file path should be given as argument, alternatively a ns can be added
    params = rosparam.load_file(rospack.get_path('mros1_reasoner')+'/config/nav_config.yaml', '')
    for param, ns in params:
        try:
            rosparam.upload_params(ns,param)
        except:
            pass # ignore empty params
    ontology_pkg = rospy.get_param('ontology_pkg')
    ontology_path = rospy.get_param('ontology_path')
    ontology_file = os.path.join(rospack.get_path(ontology_pkg)+'/'+ontology_path)
    domain_ontology_pkg = rospy.get_param('domain_ontology_pkg')
    domain_ontology_path = rospy.get_param('domain_ontology_path')
    domain_ontology_file= os.path.join(rospack.get_path(domain_ontology_pkg)+'/'+domain_ontology_path)
    function = rospy.get_param('function')
    configs = rospy.get_param('configs')
    result_file = rospy.get_param('result_file')

    tomasys = get_ontology(ontology_file).load
    onto = get_ontology(domain_ontology_file).load
    print(onto)
    rospy.loginfo('Loaded domain ontology: ' + domain_ontology_file)

    # TODO: hardcoded here that all FDs from .rossystem files solve F navigate
    #function_ = onto.search_one(iri='*'+str(function))

    #if function_ == None:
    #    print('The domain ontology provided does not contain a Function ',function)
    #    sys.exit(0)
    
    for config in configs:
        config_name = config
        file_name = config_name + '.rossystem'
        file_path = os.path.join(rospack.get_path(config_name),file_name)
        parser = RosSystemModelParser(file_path)
        model = parser.parse()
        sys_name = model.system_name[0]

        # create a FunctionDesign
        fd = tomasys.FunctionDesign(sys_name, namespace=onto, solvesF=f_navigate)
            
        # create a QualityAttribute expected value for the FunctionDesign with the type indicated by the name of the param in the RosSystem and the value of the param (e.g. 0.5)
        # value is read from CSV file instead of rossystem file
        for qa_param in model.global_parameters:
            qa_string = qa_param.param_name[0].replace('qa_', '')
            value = float(row[qa_string])
            qa = tomasys.QAvalue('{0}_{1}'.format(qa_param.param_name[0], sys_name), namespace=onto, isQAtype=onto.search_one(iri='*'+qa_string), hasValue=value)
            fd.hasQAestimation.append(qa)
        
        # add QA performance from csv even if not in rossystem model.global_parameters
        value = float(row['performance'])
        qa = tomasys.QAvalue('{0}_{1}'.format('qa_performance', sys_name),     namespace=onto, isQAtype=onto.search_one(iri='*performance'), hasValue=value)
        fd.hasQAestimation.append(qa)

    # save the ontology to a file
    onto.save(file=result_file, format='rdfxml')

