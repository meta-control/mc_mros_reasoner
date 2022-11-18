import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionServer, CancelResponse
from system_modes_msgs.srv import ChangeMode
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import KeyValue

from mros2_reasoner.reasoner import Reasoner
from mros2_reasoner.tomasys import obtainBestFunctionDesign, print_ontology_status
from mros2_reasoner.tomasys import evaluateObjectives, loadKB_from_file, destroy_entity
from mros2_reasoner.tomasys import resetObjStatus, remove_objective_grounding

from mros2_msgs.action import ControlQos
from mros2_msgs.msg import QoS


class RosReasoner(Node):
    """docstring for RosComponents."""

    def __init__(self):
        super().__init__('mros2_reasoner_node')
        # This is really not very elegant, but what can we do?
        self.reasoner = Reasoner()

        self.isInitialized = False
        self.hasObjective = False
        self.mode_change_srv_call_future = None
        self.req_reconfiguration_result = None

        self.declare_parameter("model_file")
        self.declare_parameter("tomasys_file")
        self.declare_parameter("desired_configuration")
        self.declare_parameter("nfr_energy")
        self.declare_parameter("nfr_safety")
        self.declare_parameter("reasoning_rate")
        self.declare_parameter("node_name")
        self.declare_parameter("use_reconfigure_srv")

        # Read ROS parameters

        # Whether or not to use system modes reconfiguration
        #  Used mainly for testing
        self.use_reconfiguration_srv = self.check_and_read_parameter(
            "use_reconfigure_srv", True)

        # Get ontology and tomasys file paths from parameters
        model_file_arr = self.check_and_read_parameter('model_file')
        tomasys_file_arr = self.check_and_read_parameter('tomasys_file')

        self.node_name = self.check_and_read_parameter('node_name', '')

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

        # First read fixed ontologies (Tomasys + MROS)
        self.get_logger().error(str(tomasys_file_arr))
        self.reasoner.tomasys = self.read_ontology_file(tomasys_file_arr)

        # Load the application model (individuals of tomasys classes)
        self.get_logger().error(str(model_file_arr))
        self.reasoner.onto = self.read_ontology_file(model_file_arr)

        # Check if ontologies have been correctly loaded
        if self.reasoner.tomasys is None or self.reasoner.onto is None:
            self.get_logger().error("Error while reading ontology files!")
            return

        # Get desired_configuration_name from parameters
        self.set_initial_fd(
            self.check_and_read_parameter('desired_configuration'))

        timer_rate = float(
            self.check_and_read_parameter(
                'reasoning_rate', 2.0))
        self.feedback_rate = self.create_rate(timer_rate)
        self.timer = self.create_timer(
            timer_rate, self.timer_cb, callback_group=self.cb_group)

        self.isInitialized = True
        # Reasoner initialization completed
        self.get_logger().info("[RosReasoner] -- Reasoner Initialization Ok")

    def set_initial_fd(self, initial_fd):
        if not initial_fd:
            self.grounded_configuration = initial_fd
            self.get_logger().info('grounded_configuration initialized to: ' +
                                   str(self.grounded_configuration))
        else:
            self.get_logger().info('grounded_configuration set to None')
            self.grounded_configuration = None

    def objective_cancel_goal_callback(self, cancel_request):
        self.get_logger().info("Cancel Action Callback!")
        # Stop reasoning

        if (cancel_request.qos_expected is None):
            # Checks if there are previously defined objectives.
            for old_objective in self.reasoner.search_objectives():
                self.reasoner.remove_objective(old_objective.name)
                self.hasObjective = False
                return CancelResponse.ACCEPT
        else:
            if self.reasoner.remove_objective(
                    cancel_request.qos_expected.objective_id):
                self.get_logger().info("Objective Cancelled")
                self.hasObjective = False
                return CancelResponse.ACCEPT
            else:
                self.get_logger().info("Not found")
                return CancelResponse.REJECT

    def objective_action_callback(self, objective_handle):

        self.get_logger().info("Objective Action Callback!")
        # Stop reasoning
        self.hasObjective = False

        # Checks if there are previously defined objectives.
        for old_objective in self.reasoner.search_objectives():
            self.reasoner.remove_objective(old_objective.name)

        #MonitorObjective.Goal = objective_handle.request.Goal
        obj_created = self.create_objective(objective_handle.request)
        if obj_created:
            self.hasObjective = True
            while True:
                feedback_msg = ControlQos.Feedback()
                for objective in self.reasoner.search_objectives():
                    feedback_msg.qos_status.objective_id = objective.name
                    if objective.o_status is None:
                        feedback_msg.qos_status.objective_status = str(
                            "IN_PROGRESS")
                    else:
                        feedback_msg.qos_status.objective_status = str(
                            objective.o_status)

                    feedback_msg.qos_status.objective_type = objective_handle.request.qos_expected.objective_type
                    break
                fg_instance = self.reasoner.onto.search_one(solvesO=objective)
                if fg_instance is not None:
                    feedback_msg.qos_status.selected_mode = fg_instance.typeFD.name

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

    def read_ontology_file(self, ontology_file_array):
        """ Checks if an ontology file exists and reads its value
            Args:
                    ontology_file_name (string): The name of the parameter.
                    is_base_ontology (bool): True if it's the base ontology (Tomasys).
            Returns:
                    The ontology if it's readed correctly, None otherwise.
        """
        if (type(ontology_file_array) == str):
            ontology_file_array = [ontology_file_array]
        ontology_obj = None
        for ontology_file in ontology_file_array:
            if ontology_file is not None:
                ontology = loadKB_from_file(ontology_file)
                if ontology is not None:
                    self.get_logger().info("Loaded ontology: " + str(ontology_file))
                else:
                    self.get_logger().error(
                        "Failed to load ontology from: " +
                        str(ontology_file))
                    return None
                if ontology_obj:
                    ontology_obj.imported_ontologies.append(ontology)
                else:
                    ontology_obj = ontology
            else:
                self.get_logger().warning("No ontology file provided!")
                return None

        return ontology_obj

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

    def create_objective(self, goal_request):
        ##

        #new_objective = self.reasoner.get_new_tomasys_objective("obj_" + goal_request.qos_expected.objective_type, "*" + goal_request.qos_expected.objective_type)
        new_objective = self.reasoner.get_new_tomasys_objective(
            goal_request.qos_expected.objective_id,
            "*" + goal_request.qos_expected.objective_type)
        self.get_logger().info("Creating Objective {0}".format(new_objective))
        for nrf_key in goal_request.qos_expected.qos:
            new_nfr = self.reasoner.get_new_tomasys_nrf(
                "nfr_" + nrf_key.key, "*" + nrf_key.key, float(nrf_key.value))
            self.get_logger().info("Adding NFRs {}".format(new_nfr))
            new_objective.hasNFR.append(new_nfr)
        if not goal_request.qos_expected.selected_mode:
            self.set_initial_fd(None)
        else:
            self.set_initial_fd(goal_request.qos_expected.selected_mode)

        new_objective.o_status = "UNGROUNDED"

        return True

    # NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
    def initKB(self):

        self.get_logger().info(
            'KB initialization:\n' +
            '\t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety' +
            '\n \t - Searching for objectives in the owl file:')

        objectives = self.reasoner.search_objectives()
        # if no objectives in the OWL file, standard navigation objective is
        # assumed
        if objectives == []:
            self.get_logger().info('No objectives found, waiting for new Objective')
            # # # Function Groundings and Objectives
        elif len(objectives) == 1:
            self.get_logger().info(
                'Objective {} found'.format(
                    objectives[0].name))
            self.hasObjective = True
        else:
            self.get_logger().error(
                'Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

        # For debugging InConsistent ontology errors, save the ontology before reasoning
        # self.reasoner.onto.save(file="tmp_debug.owl", format="rdfxml")

    # MVP: callback for diagnostic msg received from QA Observer

    def callbackDiagnostics(self, msg):
        if self.reasoner.onto is not None and self.hasObjective is True:
            for diagnostic_status in msg.status:
                # 2 types of diagnostics considered: about bindings in error
                # (TODO not implemented yet) or about QAs
                if diagnostic_status.message == "binding error":
                    self.get_logger().info("binding error received")
                    up_binding = self.reasoner.updateBinding(diagnostic_status)
                    if up_binding == -1:
                        self.get_logger().warning(
                            "Unkown Function Grounding: %s", diagnostic_status.name)
                    elif up_binding == 0:
                        self.get_logger().warning(
                            "Diagnostics message received for %s with level %d, nothing done about it." %
                            (diagnostic_status.name, diagnostic_status.level))

                # Component error
                elif diagnostic_status.message == "Component status":
                    # self.get_logger().warning("Component status value received \tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    up_cs = self.reasoner.updateComponentStatus(
                        diagnostic_status)
                    if up_cs == -1:
                        self.get_logger().warning("CS message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                    elif up_cs == 1:
                        self.get_logger().info(
                            "\n\nCS Message received!\tTYPE: {0}\tVALUE: {1}".format(
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))
                    else:
                        self.get_logger().warning(
                            "Unsupported CS Message received: %s ", str(
                                diagnostic_status.values[0].key))

                elif diagnostic_status.message == "QA status":
                    # self.get_logger().warning("QA value received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                    up_qa = self.reasoner.updateQA(diagnostic_status)
                    if up_qa == -1:
                        self.get_logger().warning("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                    elif up_qa == 1:
                        self.get_logger().info(
                            "QA value received!\tTYPE: {0}\tVALUE: {1}".format(
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))
                    else:
                        self.get_logger().warning(
                            "Unsupported QA TYPE received: %s ", str(
                                diagnostic_status.values[0].key))
                else:
                    self.get_logger().warning(
                        "Unsupported Message received: %s ", str(
                            diagnostic_status.values[0].key))

    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, new_configuration):

        self.get_logger().warning('New Configuration requested: {}'.format(new_configuration))
        self.req_reconfiguration_result = None

        system_modes_cli = self.create_client(
            ChangeMode,
            '/' + self.node_name + '/change_mode',
            callback_group=self.cb_group)

        while not system_modes_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Mode change service /' + self.node_name +
                                   '/change_mode not available, waiting again...')
        try:
            req = ChangeMode.Request()
            req.mode_name = new_configuration
            # async call, but the follow up while loop BLOCKS execution till
            # there is a response
            mode_change_srv_call_future = system_modes_cli.call_async(req)
            # try:
            #     rclpy.spin_until_future_complete(self, mode_change_srv_call_future)
            #     #call_result = await mode_change_srv_call_future
            # except Exception as e:
            #     self.get_logger().info('Service call failed %r' % (e,))
            # else:
            #     self.get_logger().info('Result of reconfiguration %d' % (mode_change_srv_call_future.result().success))
        except Exception as e:
            self.get_logger().info('Request creation failed %r' % (e,))
            return None
        else:
            return mode_change_srv_call_future

    # main metacontrol loop

    async def timer_cb(self):

        # self.get_logger().info('Entered timer_cb for metacontrol reasoning')
        # If we're waiting for a response from the reconfiguration, nothing
        # should be done
        if self.reasoner.isInitialized is not True:
            self.get_logger().info('Waiting to initialize Reasoner -  Nothing else will be done')
            return
        if self.hasObjective is not True:
            return

        # PRINT system status
        print_ontology_status(self.reasoner.tomasys)

        self.get_logger().info('  >> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if not self.reasoner.perform_reasoning():
            self.get_logger().error('    >> Reasoning error')
            self.reasoner.onto.save(
                file="error_reasoning.owl", format="rdfxml")
            return

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(
            self.reasoner.search_objectives())
        if not objectives_internal_error:
            self.get_logger().info("  >> No Objectives in ERROR: no adaptation is needed")
            # self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')
            # self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return
        elif len(objectives_internal_error) > 1:
            self.get_logger().error("- More than 1 objective in error, case not supported yet.")
            self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')
            self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return
        else:
            for obj_in_error in objectives_internal_error:
                self.get_logger().warning(
                    "Objective {0} in status: {1}".format(
                        obj_in_error.name, obj_in_error.o_status))
            #self.get_logger().info('  >> Finished MAPE-K ** ANALYSIS **')

        # ADAPT MAPE -Plan & Execute
        self.get_logger().info('  >> Started MAPE-K ** PLAN adaptation **')

        new_grounded = None

        if obj_in_error.o_status in ["UPDATABLE"]:
            self.get_logger().info("  >> UPDATABLE objective - Try to clear Components status")
            for comp_inst in list(
                    self.reasoner.tomasys.ComponentState.instances()):
                if comp_inst.c_status == "RECOVERED":
                    self.get_logger().info("Component {0} Status {1} - Setting to None".format(
                        comp_inst.name, comp_inst.c_status))
                    comp_inst.c_status = None

        if obj_in_error.o_status in ["UNGROUNDED"]:
            self.get_logger().info("  >>  UNGROUNDED objective - Try to set FD {0}".format(
                self.grounded_configuration))
            new_grounded = self.reasoner.set_new_grounding(
                self.grounded_configuration, obj_in_error)

        if not new_grounded:
            self.get_logger().info("  >> Reasoner searches an FD ")
            new_grounded = obtainBestFunctionDesign(
                obj_in_error, self.reasoner.tomasys)

        if not new_grounded:
            self.get_logger().error(
                "No FD found to solve Objective {} ".format(
                    obj_in_error.name))  # for DEBUGGING in csv
            # resetObjStatus(obj_in_error, "UNREACHABLE")
            # self.reasoner.set_new_grounding(None, obj_in_error)
            self.get_logger().info('Exited timer_cb for metacontrol reasoning')
            return

        # request new configuration
        self.get_logger().info('  >> Started MAPE-K ** EXECUTION **')

        if self.use_reconfiguration_srv:

            rec_result = self.request_configuration(new_grounded)

            try:
                call_result = await rec_result
            except Exception as e:
                self.get_logger().info('Reconfiguration request call failed %r' % (e,))
                return

            self.get_logger().info(
                'Got Reconfiguration result {}'.format(
                    call_result.success))
            # Process adaptation feedback to update KB:
            if (call_result.success is not None) and (
                    call_result.success is True):  # reconfiguration executed ok
                # updates the ontology according to the result of the
                # adaptation action - destroy fg for Obj and create the newly
                # grounded one
                self.grounded_configuration = self.reasoner.set_new_grounding(
                    new_grounded, obj_in_error)  # Set new grounded_configuration
            else:
                self.get_logger().error("= RECONFIGURATION FAILED =")  # for DEBUGGING in csv
                return
        else:
            self.grounded_configuration = self.reasoner.set_new_grounding(
                new_grounded, obj_in_error)  # Set new grounded_configuration

        self.get_logger().info(
            'Exited timer_cb after successful reconfiguration - Obj set to None')
