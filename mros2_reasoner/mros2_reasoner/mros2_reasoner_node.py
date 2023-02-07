import rclpy
from mros2_reasoner.ros_reasoner import RosReasoner
from rclpy.executors import MultiThreadedExecutor


def main(args=None):

    # Start rosnode
    rclpy.init(args=args)

    ros_reasoner = RosReasoner()

    # Use a MultiThreadedExecutor to enable processing service request
    # concurrently
    mt_executor = MultiThreadedExecutor()

    if ros_reasoner.is_initialized is not True:
        ros_reasoner.get_logger().info(
            "There was an error in the reasoner initialization")
        return

    # Spin until the process in terminated
    rclpy.spin(ros_reasoner, executor=mt_executor)
    ros_reasoner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
