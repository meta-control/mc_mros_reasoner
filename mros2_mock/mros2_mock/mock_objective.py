#!/usr/bin/env python
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue


class MockNode(Node):

    def __init__(self):
        super().__init__('mock')

        self._action_client = ActionClient(self, ControlQos, 'mros_objective')

        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10)

        timer_period = 4.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.mockiness_level = 1

    def set_objectives(self):
        self.get_logger().info('Waiting for server')
        self._action_client.wait_for_server()

        mock_goal_msg = ControlQos.Goal()

        mock_goal_msg.qos_expected.objective_type = "f_mock"
        mock_goal_msg.qos_expected.objective_id = "obj_mock_{:.0f}".format(
            self.get_clock().now().to_msg().sec / 10)
        mock_goal_msg.qos_expected.selected_mode = ""
        nfr = KeyValue()
        nfr.key = "mockiness"
        nfr.value = str(0.7)
        mock_goal_msg.qos_expected.qos.append(nfr)

        self.get_logger().info(
            'Sending goal  {0}'.format(
                mock_goal_msg.qos_expected.objective_type))
        self._action_client.send_goal_async(
            mock_goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

        fake_goal_msg = ControlQos.Goal()
        fake_goal_msg.qos_expected.objective_type = "f_fake"
        fake_goal_msg.qos_expected.objective_id = "obj_fake_{:.0f}".format(
            self.get_clock().now().to_msg().sec / 10)
        fake_goal_msg.qos_expected.selected_mode = ""
        nfr2 = KeyValue()
        nfr2.key = "mockiness"
        nfr2.value = str(0.8)
        fake_goal_msg.qos_expected.qos.append(nfr2)

        self.get_logger().info(
            'Sending goal  {0}'.format(
                fake_goal_msg.qos_expected.objective_type))
        self._action_client.send_goal_async(
            fake_goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
            'Best mode: {0}'.format(
                feedback.qos_status.selected_mode))
        self.get_logger().info(
            'Solving: {0} of type {1}'.format(
                feedback.qos_status.objective_id,
                feedback.qos_status.objective_type))
        self.get_logger().info(
            'obj status: {0}'.format(
                feedback.qos_status.objective_status))
        for qos in feedback.qos_status.qos:
            self.get_logger().info(
                'QoS Status: Key: {0} - Value {1}'.format(qos.key, qos.value))

    def timer_callback(self):
        if self.mockiness_level <= 0.1:
            self.destroy_node()
            return

        diag_msg = DiagnosticArray()
        diag_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ""
        key_value = KeyValue()
        key_value.key = "mockiness"
        key_value.value = str(self.mockiness_level)
        status_msg.values.append(key_value)
        status_msg.message = "QA status"
        diag_msg.status.append(status_msg)

        self.diagnostics_publisher.publish(diag_msg)

        self.mockiness_level -= 0.1


if __name__ == '__main__':
    print("Starting detect pipeline node")

    rclpy.init(args=sys.argv)

    mock_node = MockNode()
    mock_node.set_objectives()
    rclpy.spin(mock_node)

    rclpy.shutdown()
