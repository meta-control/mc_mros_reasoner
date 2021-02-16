import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock
from rclpy.action import ActionClient
from mros2_msgs.action import ControlQos

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mros2_publish_qa_node')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        # timer_period = 2.0  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.qa_type = 'energy'
        self.qa_value = 0.3
        self._action_client = ActionClient(self, ControlQos, 'mros_objective')
        self.reconf = False
        self.recover = False


    def send_goal(self):
        goal_msg = ControlQos.Goal()

        goal_msg.qos_expected.objective_type = "f_navigate"
        goal_msg.qos_expected.objective_id = "obj_navigate_{:.0f}".format(ROSClock().now().to_msg().sec/10)
        goal_msg.qos_expected.selected_mode = ""
        nfr = KeyValue()
        nfr.key = "energy"
        nfr.value = str(0.5)
        goal_msg.qos_expected.qos.append(nfr)
        nfr = KeyValue()
        nfr.key = "safety"
        nfr.value = str(0.5)
        goal_msg.qos_expected.qos.append(nfr)
        
        self.get_logger().info('Waiting for server')
        self._action_client.wait_for_server()
        
        self.get_logger().info('Sending goal  {0}'.format(goal_msg.qos_expected.objective_type))
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.get_logger().info('Goal Sent!!!')

    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Best mode: {0}'.format(feedback.qos_status.selected_mode))
        self.get_logger().info('Solving: {0} of type {1}'.format(
            feedback.qos_status.objective_id, feedback.qos_status.objective_type))
        self.get_logger().info('obj status: {0}'.format(feedback.qos_status.objective_status))
        for qos in feedback.qos_status.qos:
            self.get_logger().info('QoS Status: Key: {0} - Value {1}'.format(qos.key, qos.value))


    def timer_callback(self):
        if self.qa_value > 1.0:
            self.destroy_node()
            return

        diag_msg = DiagnosticArray()
        # now = self.clock.now().seconds_nanoseconds
        diag_msg.header.stamp = ROSClock().now().to_msg()
        status_msg = DiagnosticStatus()
        status_msg.level = DiagnosticStatus.OK
        status_msg.name = ""
        key_value = KeyValue()
        key_value.key = str(self.qa_type)
        key_value.value = str(self.qa_value)
        status_msg.values.append(key_value)
        status_msg.message = "QA status"
        diag_msg.status.append(status_msg)

        self.publisher_.publish(diag_msg)
             

        self.qa_value += 0.015
        # self.get_logger().info('qa: {0}'.format(self.qa_value))
        if self.qa_value > 0.52:
            self.qa_value = 0.4

        if self.qa_value > 0.34:
            if self.recover is False:
                self.get_logger().info('\n\nSENDING COMPO STATUS FALSE\n\n')
                diag_msg_2 = DiagnosticArray()
                diag_msg_2.header.stamp = ROSClock().now().to_msg()
                status_msg_2 = DiagnosticStatus()
                status_msg_2.level = DiagnosticStatus.OK
                status_msg_2.name = ""
                status_msg_2.message = "Component status"
                key_value_2 = KeyValue()
                key_value_2.key = "laser_resender"
                key_value_2.value = "FALSE"
                status_msg_2.values.append(key_value_2)
                diag_msg_2.status.append(status_msg_2)
                self.publisher_.publish(diag_msg_2)
                self.recover = True
        if self.qa_value > 0.42:
            if self.reconf is False:
                self.get_logger().info('\n\nSENDING COMPO STATUS RECOVEREDn\n\n')
                diag_msg_2 = DiagnosticArray()
                diag_msg_2.header.stamp = ROSClock().now().to_msg()
                status_msg_2 = DiagnosticStatus()
                status_msg_2.level = DiagnosticStatus.OK
                status_msg_2.name = ""
                status_msg_2.message = "Component status"
                key_value_2 = KeyValue()
                key_value_2.key = "laser_resender"
                key_value_2.value = "RECOVERED"
                status_msg_2.values.append(key_value_2)
                diag_msg_2.status.append(status_msg_2)
                self.publisher_.publish(diag_msg_2)
                self.reconf = True
           
            
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.send_goal()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
