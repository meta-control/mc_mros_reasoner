import rclpy
import sys
from threading import Thread
from mros2_reasoner.ros_reasoner import RosReasoner

def main(args=None):
    print('Hi from mros2_reasoner.')
    # Start rosnode
    rclpy.init(args=args)
    ros_reasoner = RosReasoner()

    if ros_reasoner.initialized:
        # initialize KB with the ontology
        ros_reasoner.initKB()
        rclpy.spin(ros_reasoner)
    else:
        ros_reasoner.get_logger().info("There was an error in the reasoner initialization")



    rclpy.shutdown()


if __name__ == '__main__':
    main()
