import rclpy
from mros2_reasoner.fake_ros_reasoner import FakeRosReasoner
from rclpy.executors import MultiThreadedExecutor


def main(args=None):

    # Start rosnode
    rclpy.init(args=args)

    ros_reasoner = FakeRosReasoner()

    # Use a MultiThreadedExecutor to enable processing service request
    # concurrently
    mt_executor = MultiThreadedExecutor()

    if ros_reasoner.isInitialized is True:
        # initialize KB with the ontology
        ros_reasoner.initKB()
    else:
        ros_reasoner.get_logger().info("There was an error in the reasoner initialization")
        return

    # Spin until the process in terminated
    rclpy.spin(ros_reasoner, executor=mt_executor)
    ros_reasoner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
