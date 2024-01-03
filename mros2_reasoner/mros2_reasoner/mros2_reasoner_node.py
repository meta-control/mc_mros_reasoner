# Copyright 2023 Knowledge-driven Autonomous Systems Laboratory.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
