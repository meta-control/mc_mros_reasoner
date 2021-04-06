import rospy

import actionlib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from metacontrol_msgs.msg import MvpReconfigurationAction
from metacontrol_msgs.msg import MvpReconfigurationGoal
from metacontrol_msgs.msg import GraphManipulationActionAction
from metacontrol_msgs.msg import GraphManipulationActionGoal
from metacontrol_msgs.msg import GraphManipulationMessage
from metacontrol_msgs.msg import SystemState

from mros1_reasoner.reasoner import Reasoner
from mros1_reasoner.tomasys import obtainBestFunctionDesign
from mros1_reasoner.tomasys import print_ontology_status, evaluateObjectives
from mros1_reasoner.tomasys import loadKB_from_file, remove_objective_grounding
from mros1_reasoner.tomasys import destroy_entity, resetObjStatus, logging


class RosReasoner(object):
    """docstring for RosComponents."""

    def __init__(self):
        super(RosReasoner, self).__init__()
        # Start ros node
        rospy.init_node('mros1_reasoner_node', anonymous=True)

        # Initialize variables.
        self.isInitialized = False
        self.reasoner = Reasoner()

        self.hasObjective = False

        # Read ROS parameters
        # Get ontology and tomasys file paths from parameters
        model_file = self.check_and_read_parameter('~model_file')
        tomasys_file_array = self.check_and_read_parameter('~tomasys_file')

        # Get desired_configuration_name from parameters
        self.grounded_configuration = self.check_and_read_parameter(
            '~desired_configuration'
        )

        # Get reasoning rate
        timer_rate = float(self.check_and_read_parameter(
            '~reasoning_rate', 2.0)
        )

        # Whether or not to use system modes reconfiguration / just for testing
        self.use_reconfiguration_srv = self.check_and_read_parameter(
            "~use_reconfigure_srv", True
        )

        if self.use_reconfiguration_srv:
            # Get Service name
            self.reconfigure_srv_name = self.check_and_read_parameter(
                '~reconfigure_srv_name', 'rosgraph_manipulator_action_server'
            )

        # Whether or not to use system modes reconfiguration / Used for testing
        self.use_reconfiguration_srv = self.check_and_read_parameter(
            "~use_reconfigure_srv", True
        )

        # Start interfaces
        rospy.Subscriber('/diagnostics',
                         DiagnosticArray,
                         self.callbackDiagnostics,)

        # First read fixed ontologies (tomasys + MROS)
        for tomasys_file in tomasys_file_array:
            if self.reasoner.tomasys is None:
                # load ontology from file
                self.reasoner.tomasys = self.read_ontology_file(tomasys_file)
            else:
                # Import additional ontology from files
                self.reasoner.tomasys.imported_ontologies.append(
                    self.read_ontology_file(tomasys_file)
                )
        # Load the application model (individuals of tomasys classes)
        self.reasoner.onto = self.read_ontology_file(model_file)
        # Check if ontologies have been correctly loaded
        if self.reasoner.tomasys is None or self.reasoner.onto is None:
            rospy.logerr("Error while reading ontology files!")
            return

        # Wait for subscribers
        # (only for the test_1_level_functional_architecture)
        # rospy.sleep(0.5)

        if self.grounded_configuration is not None:
            rospy.loginfo('grounded_configuration initialized to: %s',
                          self.grounded_configuration)
        else:
            rospy.logwarn('grounded_configuration parameter not found')

        rospy.Timer(rospy.Duration(timer_rate), self.timer_cb)

        # Reasoner initialization completed
        rospy.loginfo("[RosReasoner] -- Reasoner Initialization Ok")
        self.isInitialized = True

    @staticmethod
    def check_and_read_parameter(param_name, default_value=None):
        """ Checks if a parameter exists and returns its value
            Args:
                    param_name (string): The name of the parameter.
            Returns:
                    The parameter value if it exists, None otherwise.
        """
        if rospy.has_param(str(param_name)):
            return rospy.get_param(str(param_name))
        else:
            rospy.logwarn("Parameter \'%s\' not defined! - Returning %s",
                          str(param_name), str(default_value))
            return default_value

    def read_ontology_file(self, ontology_file_name):
        """ Checks if an ontology file exists and reads its value
            Args:
                    ontology_file_name (string): The name of the parameter.
            Returns:
                    The ontology if it's read correctly, None otherwise.
        """
        if ontology_file_name is not None:
            ontology = loadKB_from_file(ontology_file_name)
            if ontology is not None:
                rospy.loginfo("Loaded ontology: " + str(ontology_file_name))
            else:
                rospy.logerr("Failed to load ontology from: "
                             + str(ontology_file_name))
                return None
        else:
            rospy.logwarn("No ontology file provided!")
            return None
        return ontology

    # NOTE REFACTORING: This KB initialization is completely mixed with
    # ROS interfaces, probably library should not have an initKB method,
    # but utility methods to update the a-box according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file,
    # the KB is initialized only using the OWL file
    # - If there is no Objective individual,
    # a navigation Objective is create in the KB,
    # with associated NFR(s) that are read from ros parameters
    def initKB(self):

        rospy.loginfo('KB initialization:\n'
                      + '\t - Supported QAs:\n'
                      + '\t - for Function f_navigate:'
                      + '/nfr_energy, /nfr_safety\n'
                      + '\t - If an Objective instance is not found in the owl'
                      + 'file, a default o_navigate is created.')

        objectives = self.reasoner.search_objectives()

        # if no objectives in the OWL file,
        # standard navigation objective is assumed
        if objectives == []:
            rospy.loginfo('Creating Objective o_navigateA with default NFR(s)')

            o_navigate = self.reasoner.get_new_tomasys_objective("o_navigateA",
                                                                 "*f_navigate")

            # Get ontology and tomasys file paths from parameters
            nfr_energy_value = float(self.check_and_read_parameter('~nfr_energy', 0.5))  # noqa
            nfr_safety_value = float(self.check_and_read_parameter('~nfr_safety', 0.8))  # noqa

            # Load NFR(s) in the KB
            nfr_energy = self.reasoner.get_new_tomasys_nrf("nfr_energy", "*energy", nfr_energy_value)  # noqa
            nfr_safety = self.reasoner.get_new_tomasys_nrf("nfr_safety", "*safety", nfr_safety_value)  # noqa

            # Link NFR(s) to objective
            o_navigate.hasNFR.append(nfr_energy)
            o_navigate.hasNFR.append(nfr_safety)

        elif len(objectives) == 1:
            rospy.loginfo("Objective {}".format(objectives[0].name)
                          + " found, NFR(s) and initial FG are"
                          + " generated from the OWL file")

        else:
            rospy.logerr('Metacontrol cannot handle more than one Objective'
                         + 'in the OWL file (the Root Objective)')
            return

        # # Set objective to UnGrounded
        o_navigate.o_status = "UNGROUNDED"
        self.hasObjective = True
        rospy.loginfo('Objective created and set to ungrounded')

        # For debugging InConsistent ontology errors,
        # save the ontology before reasoning
        # self.onto.save(file="tmp_debug.owl", format="rdfxml")

    # MVP: callback for diagnostic msg received from QA Observer
    def callbackDiagnostics(self, msg):
        if self.reasoner.onto is not None and self.hasObjective is True:
            for diagnostic_status in msg.status:
                # 2 types of diagnostics considered: about bindings in error
                # TODO not implemented yet) or about QAs
                if diagnostic_status.message == "binding error":
                    rospy.loginfo("binding error received")
                    up_binding = self.reasoner.updateBinding(diagnostic_status)
                    if up_binding == -1:
                        rospy.logwarn("Unknown Function Grounding: %s",
                                      diagnostic_status.name)
                    elif up_binding == 0:
                        rospy.logwarn("Diagnostics message received for %s"
                                      + "with level %d , nothing done about it"
                                      % (diagnostic_status.name,
                                         diagnostic_status.level))

                # Component error
                elif diagnostic_status.message == "Component status":
                    up_cs = self.reasoner.updateComponentStatus(diagnostic_status)   # noqa
                    if up_cs == -1:
                        rospy.logwarn("CS message refers to a FG not found in"
                                      + "The KB \n we assume it refers to the"
                                      + "current grounded_configuration \n"
                                      + "(1st fg found in the KB)")
                    elif up_cs == 1:
                        rospy.loginfo(
                            "\n\nCS Message received!\tTYPE: {0}\tVALUE: {1}"
                            .format(diagnostic_status.values[0].key,
                                    diagnostic_status.values[0].value))
                    else:
                        rospy.logwarn("Unsupported CS Message received: %s ",
                                      str(diagnostic_status.values[0].key))

                # QA Status update
                elif diagnostic_status.message == "QA status":
                    rospy.loginfo(
                        "QA value received for: {0}\tTYPE: {1}\tVALUE: {2}"
                        .format(diagnostic_status.name,
                                diagnostic_status.values[0].key,
                                diagnostic_status.values[0].value))

                    up_qa = self.reasoner.updateQA(diagnostic_status)
                    if up_qa == -1:
                        rospy.logwarn("No FG found - Discarding QA message")
                    elif up_qa == 1:
                        rospy.loginfo(
                            "QA value received!\tTYPE: {0}\tVALUE: {1}"
                            .format(diagnostic_status.values[0].key,
                                    diagnostic_status.values[0].value))
                    else:
                        rospy.logwarn("Unsupported QA TYPE received: {}"
                                      .format(diagnostic_status.values[0].key))
                else:
                    rospy.logwarn("Unsupported Message received: {}"
                                  .format(diagnostic_status.values[0].key))

    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, new_configuration):

        rospy.loginfo('New Configuration requested: {}'.format(new_configuration))  # noqa
        result = None

        goal = MvpReconfigurationGoal()
        goal.desired_configuration_name = new_configuration

        rosgraph_manipulator_client = actionlib.SimpleActionClient(
                self.reconfigure_srv_name,
                MvpReconfigurationAction)

        attempts = 0
        while (
            not rosgraph_manipulator_client.wait_for_server(
                timeout=rospy.Duration(1.0))
            and attempts < 5
        ):
            rospy.logwarn("Action server not found, waiting {}s more"
                          .format(5-attempts))
            attempts += 1

        if attempts == 5:
            rospy.logerr("Action server not found, Aborting reconfiguration!")
            return None

        try:
            rosgraph_manipulator_client.send_goal(goal)
            goal_completed = rosgraph_manipulator_client.wait_for_result()
        except Exception as e:
            rospy.logerr('Request creation failed %r' % (e,))
            return None
        else:
            if not goal_completed:
                rospy.logwarn("result not found")
                return None
            result = rosgraph_manipulator_client.get_result().result
        rospy.loginfo('Got Reconfiguration result {}'.format(result))

        return result

    # main metacontrol loop
    def timer_cb(self, event):

        rospy.loginfo('Entered timer_cb for metacontrol reasoning')
        # If we're waiting for a response from the reconfiguration,
        # nothing should be done
        if self.reasoner.isInitialized is not True:
            rospy.loginfo('Waiting to initialize Reasoner ')
            return
        if self.hasObjective is not True:
            rospy.loginfo('Waiting to initialize Objective ')
            return

        # PRINT system status
        print_ontology_status(self.reasoner.tomasys)

        # EXEC REASONING to update ontology with inferences
        if not self.reasoner.perform_reasoning():
            rospy.logerr("Reasoning error")
            self.reasoner.onto.save(file="error_reasoning.owl", format="rdfxml")  # noqa

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(self.reasoner.search_objectives())  # noqa

        if not objectives_internal_error:
            rospy.loginfo("No Objectives in status ERROR: no adaptation is needed")  # noqa
            return
        elif len(objectives_internal_error) > 1:
            rospy.logerr("More than 1 objective in error, not supported yet.")
            return
        else:
            for obj_in_error in objectives_internal_error:
                rospy.logwarn("Objective {0} in status {1}"
                              .format(obj_in_error.name, obj_in_error.o_status)
                              )

        # ADAPT MAPE -Plan & Execute
        rospy.loginfo('\t>> Started MAPE-K ** PLAN adaptation **')

        new_grounded = None

        # Special cases

        # Recover from failure in component.
        if obj_in_error.o_status in ["UPDATABLE"]:
            rospy.loginfo("\t>> UPDATABLE objective - Clear Components status")
            for comp_inst in list(self.reasoner.tomasys.ComponentState.instances()):  # noqa
                if comp_inst.c_status == "RECOVERED":
                    rospy.loginfo("Component {0} Status {1} - Setting to None"
                                  .format(comp_inst.name, comp_inst.c_status))
                    comp_inst.c_status = None

        # Ungrounded objective
        if obj_in_error.o_status in ["UNGROUNDED"]:
            rospy.loginfo("\t>>  UNGROUNDED objective")
            if self.grounded_configuration is not None:
                rospy.loginfo("\t\t>>  Trying to set to initial FD {0}"
                              .format(self.grounded_configuration))
                new_grounded = self.reasoner.set_new_grounding(
                    self.grounded_configuration, obj_in_error
                )

        # Search for a new configuration
        if not new_grounded:
            rospy.loginfo("  >> Reasoner searches an FD ")
            new_grounded = obtainBestFunctionDesign(obj_in_error, self.reasoner.tomasys)  # noqa

        if not new_grounded:
            rospy.logerr("No FD found to solve Objective {} ".format(obj_in_error.name))  # noqa
            return

        # request new configuration
        rospy.loginfo('  >> Started MAPE-K ** EXECUTION **')

        if self.use_reconfiguration_srv:

            rec_result = self.request_configuration(new_grounded)

            # Process adaptation feedback to update KB:
            if rec_result is not None and rec_result != -1:
                # updates the ontology according to the result of the
                # adaptation action
                self.grounded_configuration = self.reasoner.set_new_grounding(
                    new_grounded, obj_in_error)
            else:
                rospy.logerr("= RECONFIGURATION FAILED =")
                return
        else:
            # Set new grounded_configuration
            self.grounded_configuration = self.reasoner.set_new_grounding(
                new_grounded, obj_in_error)

        rospy.loginfo('Exited timer_cb after successful reconfiguration')
