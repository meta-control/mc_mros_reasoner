import rclpy
from rclpy.node import Node

from system_modes_msgs.srv import ChangeMode
from mros2_msgs.srv import MetacontrolFD
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# TODO: Make everything fit together: node names, mode names, function_design
# names. Currently there is only one node available and the fd names do not
# match with the system_modes. Files to change: sytem_modes config file, the
# dict of this node, the ontology. 

class BridgeService(Node):
    def __init__(self):
        super().__init__('System_mode_bridge')
        self.srv = self.create_service(
                MetacontrolFD,
                '/ros_reasoner/change_node_mode',
                self.change_mode_cb)
        # Dictionary to map fd names to system_mode modes and nodes
        self.system_modes_mapping = {('f_generate_search_path','fd_spiral_low'): ('spiral_lc_node', 'SLOW'),
                ('f_generate_search_path','fd_spiral_medium'): ('spiral_lc_node', '__DEFAULT__'),
                ('f_generate_search_path','fd_spiral_high'): ('spiral_lc_node', 'FAST')
                }
    
    def change_mode_cb(self, request, response):
        # Get node name and mode name from dictionary
        f_and_fd = (request.required_function_name, request.required_fd_name)
        if f_and_fd in self.system_modes_mapping:
            (node_name, mode_name) = self.system_modes_mapping[f_and_fd]
        else:
            (node_name, mode_name) = f_and_fd

        self.get_logger().info('requested node name is {}'.format(node_name))
        self.get_logger().info('requested mode name is {}'.format(mode_name))

        system_modes_cli = self.create_client(
                ChangeMode,
                '/' + node_name + '/change_mode',
                callback_group=ReentrantCallbackGroup()) 

        try:
            req = ChangeMode.Request()
            req.mode_name = mode_name

            system_modes_response = system_modes_cli.call(req)

            response.success = system_modes_response.success
            self.get_logger().info(
                    'Response received: {}'.format(response.success))
            
        except Exception as e:
            self.get_logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return response

def main():
    rclpy.init()

    bridge_service = BridgeService()
    
    mt_executor = MultiThreadedExecutor()

    rclpy.spin(bridge_service, executor=mt_executor)
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
