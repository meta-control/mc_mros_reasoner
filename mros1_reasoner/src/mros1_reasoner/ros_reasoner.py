import rospy

import actionlib
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from metacontrol_msgs.msg import MvpReconfigurationAction, MvpReconfigurationGoal, \
                                GraphManipulationActionAction,  GraphManipulationActionGoal, \
                                GraphManipulationMessage, SystemState

from mros1_reasoner.reasoner import Reasoner
from mros1_reasoner.tomasys import obtainBestFunctionDesign, print_ontology_status, evaluateObjectives, updateGrounding, resetKBstatuses



class RosReasoner(Reasoner):
    """docstring for RosComponents."""

    def __init__(self):
        super(RosReasoner, self).__init__()
        self.initialized = False
        # Start rosnode
        rospy.init_node('mros1_reasoner_node', anonymous=True)

        #### Read ROS parameters
        # Get ontology and tomasys file paths from parameters
        model_file = self.check_and_read_parameter('~model_file')
        tomasys_file =  self.check_and_read_parameter('~tomasys_file')
        # Get desired_configuration_name from parameters
        self.grounded_configuration = self.check_and_read_parameter('~desired_configuration')

        #Start interfaces
        sub_diagnostics = rospy.Subscriber('/diagnostics', DiagnosticArray, self.callbackDiagnostics)

        self.rosgraph_manipulator_client = actionlib.SimpleActionClient(
                'rosgraph_manipulator_action_server',
                MvpReconfigurationAction)
        # rosgraph_manipulator_client.wait_for_server()

        # load tomasys
        if tomasys_file is not None:
            self.load_tomasys_from_file(tomasys_file)
            if self.tomasys is not None:
                rospy.loginfo("Loaded tomasys: %s", str(tomasys_file))
            else:
                rospy.logerr("Failed to load tomasys from: %s", str(tomasys_file))
                return
        else:
                return

        # load ontology
        if model_file is not None:
            self.load_onto_from_file(model_file)
            if self.onto is not None:
                rospy.loginfo("Loaded ontology: %s", str(model_file))
            else:
                rospy.logerr("Failed to load ontology from: %s", str(model_file))
                return
        else:
                return


        if self.grounded_configuration is not None:
            rospy.loginfo('grounded_configuration initialized to: %s', self.grounded_configuration)
        else:
            rospy.logwarn('grounded_configuration not found in the param server')

        self.initialized = True
        # Reasoner initialization completed
        rospy.loginfo("[RosReasoner] -- Reasoner Initialization Ok")


    def start_timer(self):
        timer_rate =  self.check_and_read_parameter('~reasoning_rate', 2.0)
        timer = rospy.Timer(rospy.Duration(timer_rate), self.timer_cb)


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
            rospy.logwarn("Parameter \'%s\' not defined! - Returning %s", str(param_name), str(default_value))
            return default_value

    # NOTE REFACTORING: This KB initialization is completely mixed with ROS interfaces, probably libraty should not have an initKB method, but utility methods to update the abox according to incoming information
    # Initializes the KB according to 2 cases:
    # - If there is an Objective individual in the ontology file, the KB is initialized only using the OWL file
    # - If there is no Objective individual, a navigation Objective is create in the KB, with associated NFRs that are read frmo rosparam
    def initKB(self):

        rospy.loginfo('KB initialization:\n \t - Supported QAs: \n \t \t - for Function f_navigate: /nfr_energy, /nfr_safety \n \t - If an Objective instance is not found in the owl file, a default o_navigate is created.' )

        objectives = self.search_objectives()

        # if no objectives in the OWL file, standard navigation objective is assumed
        if objectives == []:
            rospy.loginfo('Creating default Objective o_navigateA with default NFRs')

            obj_navigate = self.get_new_tomasys_objective("o_navigateA", "*f_navigate")

            # Get ontology and tomasys file paths from parameters
            nfr_energy_value = float(self.check_and_read_parameter('~nfr_energy', 0.5))
            nfr_safety_value = float(self.check_and_read_parameter('~nfr_safety', 0.8))

            # Load NFRs in the KB
            nfr_energy = self.get_new_tomasys_nrf("nfr_energy", "*energy", nfr_energy_value)
            nfr_safety = self.get_new_tomasys_nrf("nfr_safety", "*safety", nfr_safety_value)

            # Link NFRs to objective
            obj_navigate.hasNFR.append(nfr_energy)
            obj_navigate.hasNFR.append(nfr_safety)

            # # Function Groundings and Objectives
            fg = self.tomasys.FunctionGrounding("fg_{}".format(self.grounded_configuration), namespace=self.onto, typeFD=self.onto.search_one(iri="*{}".format(self.grounded_configuration)), solvesO=obj_navigate)

        elif len(objectives) == 1:
            o = objectives[0]
            fg = self.tomasys.FunctionGrounding("fg_" + o.name.replace('o_',''), namespace=self.onto, typeFD=obtainBestFunctionDesign(o, self.tomasys), solvesO=o)
            rospy.logwarn('Objective, NFRs and initial FG are generated from the OWL file')
        else:
            rospy.logerr('Metacontrol cannot handle more than one Objective in the OWL file (the Root Objective)')

        # For debugging InConsistent ontology errors, save the ontology before reasoning
        self.onto.save(file="tmp_debug.owl", format="rdfxml")


    # MVP: callback for diagnostic msg received from QA Observer
    def callbackDiagnostics(self, msg):
        for diagnostic_status in msg.status:
            # 2 types of diagnostics considered: about bindings in error (TODO not implemented yet) or about QAs
            if diagnostic_status.message == "binding error":
                rospy.loginfo("binding error received")
                up_binding = self.updateBinding(diagnostic_status.name, diagnostic_status.level)
                if up_binding == -1:
                    rospy.logwarn("Unkown Function Grounding: %s", diagnostic_name)
                elif up_binding == 0:
                    rospy.logwarn("Diagnostics message received for %s with level %d, nothing done about it." % (diagnostic_name, diagnostic_level))

            if diagnostic_status.message == "QA status":
                rospy.logwarn("QA value received for\t{0} \tTYPE: {1}\tVALUE: {2}".format(diagnostic_status.name, diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                up_qa = self.updateQA(diagnostic_status)
                if up_qa == -1:
                    rospy.logwarn("QA message refers to a FG not found in the KB, we asume it refers to the current grounded_configuration (1st fg found in the KB)")
                elif up_qa == 1:
                    rospy.loginfo("QA value received!\tTYPE: {0}\tVALUE: {1}".format(diagnostic_status.values[0].key, diagnostic_status.values[0].value))
                else:
                    rospy.logwarn("Unsupported QA TYPE received: %s ", str(diagnostic_status.values[0].key))
    # for MVP with QAs - request the FD.name to reconfigure to
    def request_configuration(self, fd):
        rospy.logwarn_throttle(1., 'New Configuration requested: {}'.format(fd.name))

        goal = MvpReconfigurationGoal()
        goal.desired_configuration_name = fd.name
        self.rosgraph_manipulator_client.send_goal(goal)
        self.rosgraph_manipulator_client.wait_for_result()
        result = self.rosgraph_manipulator_client.get_result().result
        rospy.loginfo('Result: {}'.format(result) )
        return result

    ## main metacontrol loop
    def timer_cb(self, event):

        rospy.loginfo('Entered timer_cb for metacontrol reasoning')
        rospy.loginfo('  >> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if self.perform_reasoning():
            rospy.loginfo('     >> Finished ontological reasoning)')
        else:
            rospy.logerr("Reasoning error")

        # PRINT system status
        print_ontology_status(self.tomasys)

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_internal_error = evaluateObjectives(self.tomasys)
        if not objectives_internal_error:
            rospy.loginfo("No Objectives in status ERROR: no adaptation is needed")
            rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        elif len(objectives_internal_error) > 1 :
            rospy.logerr("- More than 1 objectives in error, case not supported yet.")
            rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')
            rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        else:
            rospy.logwarn("Objectives in status ERROR: {}".format([o.name for o in objectives_internal_error]) )
            rospy.loginfo('  >> Finished MAPE-K ** ANALYSIS **')

        # ADAPT MAPE -Plan & Execute
        rospy.loginfo('  >> Started MAPE-K ** PLAN adaptation **')


        o = objectives_internal_error[0]
        rospy.loginfo("=> Reasoner searches FD for objective: {}".format(o.name) )
        fd = obtainBestFunctionDesign(o, self.tomasys)
        if not fd:
            rospy.logerr(
                "No FD found to solve Objective {}".format(o.name)) # for DEBUGGING in csv
            rospy.loginfo('Exited timer_cb for metacontrol reasoning')
            return
        rospy.loginfo('  >> Finished MAPE-K ** Plan adaptation **')

        # request new configuration
        rospy.loginfo('  >> Started MAPE-K ** EXECUTION **')
        result = self.request_configuration(fd)
        rospy.loginfo('  >> Finished MAPE-K ** EXECUTION **')
        # Process adaptation feedback to update KB:
        if result == 1: # reconfiguration executed ok
            rospy.logwarn("= RECONFIGURATION SUCCEEDED =") # for DEBUGGING in csv
            # updates the ontology according to the result of the adaptation action - destroy fg for Obj and create the newly grounded one
            grounded_configuration = updateGrounding(o, fd, self.tomasys, self.onto) # Set new grounded_configuration
            resetKBstatuses(self.tomasys)
        elif result == -1:
            rospy.logerr("= RECONFIGURATION UNKNOWN =") # for DEBUGGING in csv
        else:
            rospy.logerr("= RECONFIGURATION FAILED =") # for DEBUGGING in csv
        rospy.loginfo('Exited timer_cb for metacontrol reasoning')
