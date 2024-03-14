import rclpy
from mros2_reasoner.ros_reasoner import RosReasoner
from rclpy.executors import MultiThreadedExecutor


def main(args=None):

    # Start rosnode
    rclpy.init(args=args)

    try:
        ros_reasoner = RosReasoner()
        executor = MultiThreadedExecutor()
        executor.add_node(ros_reasoner)

        if ros_reasoner.is_initialized is not True:
            ros_reasoner.get_logger().info(
                "There was an error in the reasoner initialization")
            return

        try:
            executor.spin()
        except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
            executor.shutdown()
            ros_reasoner.destroy_node()
        finally:
            executor.shutdown()
            ros_reasoner.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
