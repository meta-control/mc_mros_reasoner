from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from mros2_msgs.action import ControlQos
from mros2_msgs.msg import QoS


class FakeRosReasoner(Node):
    """docstring for RosComponents."""

    def __init__(self):
        super().__init__('mros2_reasoner_node')

        self.isInitialized = False
        self.hasObjective = False
        self.declare_parameter("desired_configuration")
        self.declare_parameter("reasoning_rate")
        self.declare_parameter("node_name")

        # Read ROS parameters

        self.node_name = self.check_and_read_parameter('node_name', 'pilot')

        self.cb_group = ReentrantCallbackGroup()

        # Start interfaces
        # subscriptions for different message types (named, pins, angle)
        # Node's default callback group is mutually exclusive. This would prevent the change mode
        # requets' response from being processed until the timer callback finished.
        # The callback_group should solve this.

        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.callbackDiagnostics,
            1,
            callback_group=self.cb_group)
        # Create action server
        self.objective_action_server = ActionServer(
            self,
            ControlQos,
            'mros_objective',
            self.objective_action_callback,
            callback_group=self.cb_group,
            cancel_callback=self.objective_cancel_goal_callback)

        # Get desired_configuration_name from parameters

        self.grounded_configuration = self.check_and_read_parameter(
            'desired_configuration')
        self.get_logger().info('grounded_configuration initialized to: ' +
                               str(self.grounded_configuration))

        timer_rate = float(
            self.check_and_read_parameter(
                'reasoning_rate', 2.0))
        self.feedback_rate = self.create_rate(timer_rate)

        self.isInitialized = True
        # Reasoner initialization completed
        self.get_logger().info("[RosReasoner] -- Reasoner Initialization Ok")

    def objective_cancel_goal_callback(self, cancel_request):
        self.get_logger().info("Cancel Action Callback!")
        self.get_logger().info("Objective Cancelled")
        self.hasObjective = False
        return CancelResponse.ACCEPT

    def objective_action_callback(self, objective_handle):

        # Check if there's a previous goal running.
        if self.hasObjective is True:
            self.get_logger().info("A previous action was found!")
            self.feedback_rate.sleep()

        self.get_logger().info("Objective Action Callback!")
        # Stop reasoning
        self.hasObjective = True
        while True:
            feedback_msg = ControlQos.Feedback()
            feedback_msg.qos_status.objective_id = objective_handle.request.qos_expected.objective_id
            feedback_msg.qos_status.objective_status = str("IN_PROGRESS")
            feedback_msg.qos_status.objective_type = objective_handle.request.qos_expected.objective_type
            feedback_msg.qos_status.selected_mode = self.grounded_configuration
            # QAValue = KeyValue()
            #         QAValue.key = str(qa.isQAtype.name)
            #         QAValue.value = str(qa.hasValue)
            #         feedback_msg.qos_status.qos.append(QAValue)
            if self.hasObjective is False:
                self.get_logger().info("Cancel goal!")
                break
            objective_handle.publish_feedback(feedback_msg)
            self.feedback_rate.sleep()
        objective_handle.succeed()
        return ControlQos.Result()

    def check_and_read_parameter(self, param_name, default_value=None):
        """ Checks if a parameter exists and returns its value
            Args:
                    param_name (string): The name of the parameter.
            Returns:
                    The parameter value if it exists, None otherwise.
        """
        # Helper function to return value of a parameter
        if self.has_parameter(param_name):
            param_desc = self.get_parameter(param_name)
            if param_desc.type_ == Parameter.Type.NOT_SET:
                ret = default_value
            else:
                ret = param_desc.value
        else:
            self.get_logger().warning(
                'Fetch of parameter that does not exist: ' +
                param_name +
                ' - Returning ' +
                str(default_value))
            ret = default_value

        return ret

    # NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam

    def initKB(self):

        self.get_logger().info(
            'KB initialization:\n' +
            '\t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety' +
            '\n \t - Searching for objectives in the owl file:')

        self.get_logger().info('No objectives found, waiting for new Objective')
        # For debugging InConsistent ontology errors, save the ontology before reasoning
        # self.reasoner.onto.save(file="tmp_debug.owl", format="rdfxml")

    # MVP: callback for diagnostic msg received from QA Observer

    def callbackDiagnostics(self, msg):
        for diagnostic_status in msg.status:
            # 2 types of diagnostics considered: about bindings in error (TODO
            # not implemented yet) or about QAs
            if diagnostic_status.message == "binding error":
                self.get_logger().warning(
                    "Diagnostics message received for %s with level %d, nothing done about it." %
                    (diagnostic_status.name, diagnostic_status.level))

            # Component error
            elif diagnostic_status.message == "Component status":
                # self.get_logger().warning("Component status value received \tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                self.get_logger().info(
                    "\nCS Message received!\tTYPE: {0}\tVALUE: {1}".format(
                        diagnostic_status.values[0].key,
                        diagnostic_status.values[0].value))
                # request new configuration
                component = diagnostic_status.values[0].key
                value = diagnostic_status.values[0].value
                self.get_logger().info(
                    "Component: {0} - Value {1}".format(component, value))
                if component == "battery":
                    # request new configuration
                    if value.upper() == "FALSE":
                        self.get_logger().info("CS Message Battery -  False")
                # request new configuration
                        self.grounded_configuration = "f_energy_saving_mode"
                    elif value.upper() == "RECOVERED":
                        self.get_logger().info("CS Message Battery -  True")
                        self.grounded_configuration = self.check_and_read_parameter(
                            'desired_configuration')
                        # request new configuration
                elif component == "laser_resender":
                    if value.upper() == "FALSE":
                        self.get_logger().info("CS Messagge laser - False")
                        self.grounded_configuration = "f_degraded_mode"
                    elif value.upper() == "RECOVERED":
                        self.get_logger().info("CS Messagge laser - True")
                        self.grounded_configuration = self.check_and_read_parameter(
                            'desired_configuration')
                else:
                    self.get_logger().warning(
                        "Unsupported CS Message received: %s ", str(
                            diagnostic_status.values[0].key))
            elif diagnostic_status.message == "QA status":
                self.get_logger().info(
                    "QA value received!\tTYPE: {0}\tVALUE: {1}".format(
                        diagnostic_status.values[0].key,
                        diagnostic_status.values[0].value))
