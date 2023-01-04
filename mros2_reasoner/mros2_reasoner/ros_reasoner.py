import rclpy

from rclpy.action import ActionServer
from rclpy.action import CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.parameter import Parameter

from system_modes_msgs.srv import ChangeMode
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue

from mros2_reasoner.reasoner import Reasoner

from mros2_msgs.action import ControlQos
from mros2_msgs.msg import QoS
from mros2_msgs.srv import MetacontrolFD


class RosReasoner(Node, Reasoner):

    def __init__(self):
        Node.__init__(self, 'mros2_reasoner_node')

        self.declare_parameter('model_file', Parameter.Type.STRING)
        self.declare_parameter('tomasys_file', Parameter.Type.STRING_ARRAY)

        # Get ontology and tomasys file paths from parameters
        Reasoner.__init__(
            self,
            self.get_parameter('tomasys_file').value,
            self.get_parameter('model_file').value
        )

        self.declare_parameter('desired_configuration', Parameter.Type.STRING)
        self.declare_parameter('node_name', '')
        self.declare_parameter('reasoning_period', 5)
        self.declare_parameter('use_reconfigure_srv', True)

        # Whether or not to use system modes reconfiguration
        #  Used mainly for testing
        self.use_reconfiguration_srv = self.get_parameter(
            'use_reconfigure_srv').value

        # Use execute_ros instead of Reasoner.execute
        if self.use_reconfiguration_srv:
            self.execute = self.execute_ros

        self.is_initialized = False
        self.mode_change_srv_future = None
        self.req_reconfiguration_result = None

        self.node_name = self.get_parameter('node_name').value

        self.cb_group = ReentrantCallbackGroup()

        # Start interfaces
        # subscriptions for different message types (named, pins, angle)
        # Node's default callback group is mutually exclusive. This would
        # prevent the change mode requests' response from being processed until
        # the timer callback finished. The callback_group should solve this.
        self.diganostic_sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
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
        self.set_initial_fd(self.get_parameter('desired_configuration').value)

        timer_rate = self.get_parameter('reasoning_period').value

        self.feedback_rate = self.create_rate(timer_rate)
        self.metacontrol_loop_timer = self.create_timer(
            timer_rate,
            self.metacontrol_loop_callback,
            callback_group=self.cb_group)

        self.logger = self.get_logger()

        # Reasoner initialization completed
        self.is_initialized = True
        self.logger.info('[RosReasoner] -- Reasoner Initialization Ok')

    def set_initial_fd(self, initial_fd):
        if initial_fd != '':
            self.grounded_configuration = initial_fd
            self.logger.info('grounded_configuration initialized to: ' +
                             str(self.grounded_configuration))
        else:
            self.logger.info('grounded_configuration set to None')
            self.grounded_configuration = None

    def objective_cancel_goal_callback(self, cancel_request):
        self.logger.info('Cancel Action Callback!')
        # Stop reasoning

        if (cancel_request.qos_expected is None):
            # Checks if there are previously defined objectives.
            for old_objective in self.search_objectives():
                self.remove_objective(old_objective.name)
                return CancelResponse.ACCEPT
        else:
            if self.remove_objective(
                    cancel_request.qos_expected.objective_id):
                self.logger.info('Objective Cancelled')
                return CancelResponse.ACCEPT
            else:
                self.logger.info('Not found')
                return CancelResponse.REJECT

    def objective_action_callback(self, objective_handle):

        self.logger.info('Objective Action Callback!')
        # Stop reasoning

        # TODO: objectives won't have same name
        # Checks if there are previously defined objectives.
        for old_objective in self.search_objectives():
            self.remove_objective(old_objective.name)

        obj_created = self.create_objective(objective_handle.request)
        if obj_created:
            while True:
                feedback_msg = ControlQos.Feedback()
                for objective in self.search_objectives():
                    feedback_msg.qos_status.objective_id = objective.name
                    if objective.o_status is None:
                        feedback_msg.qos_status.objective_status = str(
                            'IN_PROGRESS')
                    else:
                        feedback_msg.qos_status.objective_status = str(
                            objective.o_status)

                    feedback_msg.qos_status.objective_type = \
                        objective_handle.request.qos_expected.objective_type
                    break
                fg_instance = self.onto.search_one(solvesO=objective)
                if fg_instance is not None:
                    feedback_msg.qos_status.selected_mode = \
                        fg_instance.typeFD.name

                    for qa in fg_instance.hasQAvalue:
                        QAValue = KeyValue()
                        QAValue.key = str(qa.isQAtype.name)
                        QAValue.value = str(qa.hasValue)
                        feedback_msg.qos_status.qos.append(QAValue)
                else:
                    if objective is None:
                        objective_handle.canceled()
                        break

                objective_handle.publish_feedback(feedback_msg)

                self.feedback_rate.sleep()
            objective_handle.succeed()
        else:
            objective_handle.fail()

        return ControlQos.Result()

    def create_objective(self, goal_request):
        new_objective = self.get_new_tomasys_objective(
            goal_request.qos_expected.objective_id,
            "*" + goal_request.qos_expected.objective_type)
        self.logger.info('Creating Objective {0}'.format(new_objective))
        for nfr_key in goal_request.qos_expected.qos:
            nfr_id = \
                goal_request.qos_expected.objective_id + '_nfr_' + nfr_key.key
            new_nfr = self.get_new_tomasys_nfr(
                 nfr_id, nfr_key.key, float(nfr_key.value))
            self.logger.info('Adding NFRs {}'.format(new_nfr))
            new_objective.hasNFR.append(new_nfr)

        # TODO: this is not working
        if not goal_request.qos_expected.selected_mode:
            self.set_initial_fd(None)
        else:
            self.set_initial_fd(goal_request.qos_expected.selected_mode)

        # TODO: shouldn't this be a swrl rule instead of hardcoded?
        new_objective.o_status = 'UNGROUNDED'

        return True

    # MVP: callback for diagnostic msg received from QA Observer
    def diagnostics_callback(self, msg):
        if self.onto is not None and self.has_objective() is True:
            for diagnostic_status in msg.status:
                if diagnostic_status.message == 'binding error':
                    self.logger.info('binding error received')
                    up_binding = self.update_binding(diagnostic_status)
                    if up_binding == -1:
                        self.logger.warning(
                            'Unkown Function Grounding: %s',
                            diagnostic_status.name)
                    elif up_binding == 0:
                        self.logger.warning(
                            'Diagnostics message received for %s' +
                            ' with level %d, nothing done about it.' %
                            (diagnostic_status.name, diagnostic_status.level))

                # Component error
                elif diagnostic_status.message == "Component status":
                    up_cs = self.update_component_status(
                        diagnostic_status)
                    if up_cs == -1:
                        self.logger.warning(
                            'CS message refers to a FG not found in the KB, ' +
                            ' we asume it refers to the current ' +
                            'grounded_configuration (1st fg found in the KB)')
                    elif up_cs == 1:
                        self.logger.info(
                            '\n\nCS Message received!' +
                            '\tTYPE: {0}\tVALUE: {1}'.format(
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))
                    else:
                        self.logger.warning(
                            'Unsupported CS Message received: %s ', str(
                                diagnostic_status.values[0].key))

                elif diagnostic_status.message == "QA status":
                    up_qa = self.update_qa(diagnostic_status)
                    if up_qa:
                        self.logger.info(
                            'QA value received!\tTYPE: {0}\tVALUE: {1}'.format(
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))
                    else:
                        self.logger.warning(
                            'Unsupported QA TYPE received: %s ', str(
                                diagnostic_status.values[0].key))
                else:
                    self.logger.warning(
                        'Unsupported Message received: %s ', str(
                            diagnostic_status.values[0].key))

    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, desired_configuration, objective):
        self.logger.warning(
            'New Configuration for objective {0} requested: {1}'.format(
                objective, desired_configuration))
        self.req_reconfiguration_result = None

        mode_change_cli = self.create_client(
                MetacontrolFD,
                '/ros_reasoner/change_node_mode',
                callback_group=self.cb_group)

        while not mode_change_cli.wait_for_service(timeout_sec=1.0):
            self.logger().warn(
                'Mode change service ' +
                '/ros_reasoner/change_node_mode not available, waiting...')

        try:
            req = MetacontrolFD.Request()
            req.required_function_name = str(objective.typeF.name)
            req.required_fd_name = desired_configuration

            mode_change_response = mode_change_cli.call(req)
        except Exception as e:
            self.logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return mode_change_response


    def execute_ros(self, desired_configurations):
        if self.has_objective() is False or desired_configurations == dict():
            return

        self.logger.info('  >> Started MAPE-K ** EXECUTION **')
        self.logger.info(
                'desired_configurations are: {}'.format(desired_configurations))
        for objective in desired_configurations:
            reconfiguration_result = self.request_configuration(
                desired_configurations[objective], objective)

            if reconfiguration_result is not None \
               and reconfiguration_result.success is True:
                self.logger.info(
                    'Got Reconfiguration result {}'.format(
                        reconfiguration_result.success))

                # Process adaptation feedback to update KB:
                self.set_new_grounding(
                    desired_configurations[objective], objective)
            else:
                self.logger.error('= RECONFIGURATION FAILED =')
                return

    # main metacontrol loop
    def metacontrol_loop_callback(self):

        if self.is_initialized is not True:
            self.logger.info(
                'Waiting to initialize Reasoner -  Nothing else will be done')
            return

        # Analyze
        objectives_in_error = self.analyze()

        # Plan
        desired_configurations = self.plan(objectives_in_error)

        # Execute
        self.execute(desired_configurations)

        self.logger.info(
            'Exited metacontrol_loop_callback')
