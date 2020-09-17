#!/usr/bin/env python
"""
author: c.h.corbato@tudelft.nl
using Owlready2 to manipulate the ontology

README
- I created a python vrit env for this: $ pew new --python=python3 owlready2
- installed owlready2 in my ws: $ pip install Owlready2
- Make sure you are working in the Virtual environment: $ pew workon owlready2
- run this script by: $ python owl_reasoner.py

"""
import rospy
from mros1_reasoner.ros_reasoner import RosReasoner


if __name__ == '__main__':

    ros_reasoner = RosReasoner()

    if ros_reasoner.initialized:
        # initialize KB with the ontology
        ros_reasoner.initKB()
        ros_reasoner.start_timer()
        rospy.spin()
    else:
        rospy.logerr("There was an error in the reasoner initialization")
