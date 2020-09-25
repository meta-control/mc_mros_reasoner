import rclpy
from rclpy.node import Node
from rclpy.clock import ROSClock


from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('mros2_publish_qa_node')
        self.publisher_ = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        timer_period = 2.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.qa_type = 'energy'
        self.qa_value = 0.3



    def timer_callback(self):
        if self.qa_value > 1.0:
            self.destroy_node()
            return

        diag_msg = DiagnosticArray()
        #now = self.clock.now().seconds_nanoseconds
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
        self.qa_value += 0.1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
