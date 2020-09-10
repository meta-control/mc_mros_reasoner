
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
# Revision $Id: gossipbot.py 1013 2008-05-21 01:08:56Z sfkwc $

## Talker/listener demo validation 

PKG = 'mros1_reasoner'
NAME = 'send_QA_value_test'
MSG_DELAY = 0.2

import sys, unittest
import rospy, rostest

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class TestSendQAValue(unittest.TestCase):
    def __init__(self, *args):
        super(TestSendQAValue, self).__init__(*args)
        self.success = False
        
    
    ############################################################################## 
    def setUp(self):
    ############################################################################## 
        rospy.init_node(NAME, anonymous=True)
        self.message_pub = rospy.Publisher(
            '/diagnostics', DiagnosticArray, queue_size=10)

    ############################################################################## 
    def test_one_equals_one(self):
    ############################################################################## 
        rospy.loginfo("-D- test_one_equals_one")
        self.assertEquals(1, 1, "1!=1")
    
    ############################################################################## 
    def send_qa_value_msgs(self, key_name, max_value, min_value, duration=10.0):
    ############################################################################## 
        sleep_rate = rospy.Rate(1.0)
        key_value = max_value        
        step_value = (max_value - min_value)/ duration

        rospy.loginfo("- D - Test sending %s -  %s - Step value %s" % (str(key_name), str(key_value), str(step_value))) 


        while not rospy.is_shutdown() and not self.success and key_value > min_value:
            
            rospy.loginfo("- D - Test sending %s -  %s" % (str(key_name), str(key_value)))
            status_msg = DiagnosticStatus()
            status_msg.level = DiagnosticStatus.OK
            status_msg.name = ""
            status_msg.values.append(
            KeyValue(str(key_name), str(key_value)))
            status_msg.message = "QA status"
        

            diag_msg = DiagnosticArray()
            diag_msg.header.stamp = rospy.get_rostime()
            diag_msg.status.append(status_msg)
            self.message_pub.publish(diag_msg)
            key_value = key_value - step_value
            sleep_rate.sleep()

    ############################################################################## 
    def test_publish_high_energy_qa_value(self):
    ############################################################################## 
        self.send_qa_value_msgs("energy", 0.8, 0.1, 10.0)
        rospy.sleep(MSG_DELAY)
        self.assertEquals(1, 1, "1!=1")
        
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestSendQAValue, sys.argv)

