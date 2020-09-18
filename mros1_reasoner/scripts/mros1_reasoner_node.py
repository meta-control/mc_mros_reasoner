#!/usr/bin/env python
###########################################
# 
# authors:    M.A.GarzonOviedo@tudelft.nl
#             c.h.corbato@tudelft.nl
##########################################


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
