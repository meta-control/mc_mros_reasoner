#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#

# Talker/listener demo validation

import sys
import unittest
import rostest
import rospy
import time


from rosgraph_msgs.msg import Log

PKG = 'mros1_reasoner'
NAME = 'test_level_1_functions'
MSG_DELAY = 0.2
TIMEOUT = 5.0


class TestLevel1Functions(unittest.TestCase):
    def __init__(self, *args):
        super(TestLevel1Functions, self).__init__(*args)
        self.success = False
        self.success_level_1 = 0

    ###########################################################################
    def test_one_equals_one(self):
        #######################################################################
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")

    ###########################################################################
    def test_1_level_functional_architecture(self):
        #######################################################################
        self.success = False
        self.success_level_1 = 0
        self.subscriber_functional_architecture(TIMEOUT)
        rospy.sleep(MSG_DELAY)
        self.assertTrue(self.success)

    ###########################################################################
    def log_callback_level_1(self, log_data):
        #######################################################################

        if log_data.name == '/reasoner':

            rospy.loginfo("In log callback !")
            if log_data.level == Log.INFO:
                if (log_data.msg.startswith("Loaded ontology:")):
                    rospy.loginfo("OWL file loaded into KB: %s" % log_data.msg)
            if log_data.level == Log.INFO:
                if (log_data.msg.startswith("[RosReasoner] -- "
                                            + "Reasoner Initialization Ok")):
                    rospy.loginfo("Reasoner initialized ok: %s" % log_data.msg)
                    self.success_level_1 = self.success_level_1 + 1
            if log_data.level == Log.INFO:
                if (log_data.msg.startswith("Objective created and set to ungrounded")):  # noqa
                    rospy.loginfo("Objective created: %s" % log_data.msg)
                    self.success_level_1 = self.success_level_1 + 1
            if log_data.level == Log.INFO:
                if (log_data.msg.startswith("Exited timer_cb after successful reconfiguration")):  # noqa
                    rospy.loginfo("Objective reconfigured: %s" % log_data.msg)
                    self.success_level_1 = self.success_level_1 + 1
            if (self.success_level_1 == 3):
                rospy.loginfo("Got the 3 correct messages! - level_functional test OK!")  # noqa
                self.success = True

    ###########################################################################
    def subscriber_functional_architecture(self, timeout=10.0):
        #######################################################################

        rospy.init_node(NAME, anonymous=True)
        rospy.loginfo("Init node")
        rospy.Subscriber("/rosout", Log, self.log_callback_level_1)

        timeout_t = time.time() + timeout  # Default timeout 5 sec.

        while (not rospy.is_shutdown()
               and not self.success and time.time() < timeout_t):
            time.sleep(MSG_DELAY)


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestLevel1Functions, sys.argv)
