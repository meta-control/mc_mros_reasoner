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

import signal
import sys
from threading import Lock

from mros2_reasoner.tomasys import ground_fd
from mros2_reasoner.tomasys import obtain_best_function_design
from mros2_reasoner.tomasys import remove_objective_grounding
from mros2_reasoner.tomasys import reset_fd_realisability
from mros2_reasoner.tomasys import reset_objective_status
from mros2_reasoner.tomasys import update_fg_measured_qa
from mros2_reasoner.tomasys import update_measured_qa_value

from mros2_reasoner.owlready2_tomasys import OwlReady2TOMASys


import logging


class Reasoner:

    def __init__(self, ontology_file_array):

        self.kb_interface = OwlReady2TOMASys()
        self.kb_interface.load_ontology_file(ontology_file_array)

        # Dictionary with request configurations for functions
        self.requested_configurations = dict()

        self.logger = logging
        signal.signal(signal.SIGINT, self.save_ontology_exit)

    def remove_objective(self, objective_id):
        # Checks if there are previously defined objectives.
        old_objective = self.onto.search_one(iri="*{}".format(objective_id))
        if old_objective:
            old_fg_instance = self.onto.search_one(solvesO=old_objective)
            with self.ontology_lock:
                destroy_entity(old_fg_instance)
                destroy_entity(old_objective)
            return True
        else:
            return False

    def get_objective_from_objective_id(self, objective_id):
        objectives = self.search_objectives()
        if objectives == []:
            return None
        for objective in objectives:
            if str(objective.name) == str(objective_id):
                return objective

    def get_function_name_from_objective_id(self, objective_id):
        objective = self.get_objective_from_objective_id(objective_id)
        if objective is None:
            return None
        else:
            return str(objective.typeF.name)

    def get_objectives_in_error(self):
        return get_objectives_in_error(self.search_objectives())

    # def get_new_tomasys_objective(self, objective_name, iri_seed):
    #     """ Creates Objective individual in the KB given a desired name and a
    #     string seed for the Function name
    #     """
    #     objective = self.tomasys.Objective(
    #         str(objective_name),
    #         namespace=self.onto,
    #         typeF=self.onto.search_one(
    #             iri=str(iri_seed)))
    #     return objective

    # def get_new_tomasys_nfr(self, qa_value_name, nfr_key, nfr_value):
    #     """Creates QAvalue individual in the KB given a desired name and a
    #     string seed for the QAtype name and the value
    #     """
    #
    #     # TODO: this search is not optimal, the search + loop can be
    #     # substituted by a single search
    #     qa_type = self.get_qa_type(nfr_key)
    #
    #     new_nfr = self.tomasys.QAvalue(
    #         str(qa_value_name),
    #         namespace=self.onto,
    #         isQAtype=qa_type,
    #         hasValue=nfr_value)
    #
    #     return new_nfr

    def set_new_grounding(self, fd_name, objective):
        """
        Set new function grounding

        Given a string fd_name with the name of a FunctionDesign and an
        objective, removes the previous fg for the objective and ground a new
        fg of typeF fd
        """
        remove_objective_grounding(objective, self.tomasys, self.onto)
        fd = self.onto.search_one(
            iri="*{}".format(fd_name),
            is_a=self.tomasys.FunctionDesign)
        if fd:
            with self.ontology_lock:
                ground_fd(fd, objective, self.tomasys, self.onto)
                reset_objective_status(objective)
            return str(fd.name)
        else:
            return None

    # the DiagnosticStatus message process contains, per field
    # - message: "binding_error"
    # - name: name of the fg reported, as named in the OWL file
    # - level: values 0 and 1 are mapped to nothing, values 2 or 3 are mapped
    # to fg.status="INTERNAL_ERROR"
    def update_binding(self, diagnostic_status):
        fg = self.onto.search_one(iri="*{}".format(diagnostic_status.name))
        if fg is None:
            return -1
        if diagnostic_status.level > 1:
            fg.fg_status = "INTERNAL_ERROR"
            return 1
        else:
            return 0

    # the DiagnosticStatus message process contains, per field
    # - message: "Component status"
    # - key: name of the Component Reported, as named in the OWL file
    # - value: (True, False ) Meaning wheter or not the component is working OK
    def update_component_status(self, diagnostic_status):
        # Find the Component with the same name that the one in the Component
        # Status message (in diagnostic_status.key)
        component_type = self.onto.search_one(
            iri="*{}".format(diagnostic_status.values[0].key))
        if component_type is not None:
            value = diagnostic_status.values[0].value
            with self.ontology_lock:
                reset_fd_realisability(
                    self.tomasys,
                    self.onto,
                    diagnostic_status.values[0].key)
                component_type.c_status = value
            return_value = 1
        else:
            return_value = 0
        return return_value

    def get_qa_type(self, key):
        # TODO: this search is not optimal, the search + loop can be
        # substituted by a single search or instances()
        qa_types = self.onto.search(type=self.tomasys.QualityAttributeType)
        qa_type = None
        for qa in qa_types:
            if qa.name == key:
                qa_type = qa
                break
        return qa_type

    # TODO: can this be optimized with a list comprehension?
    def get_function_groudings_require_qa(self, qa_key):
        fgs = self.tomasys.FunctionGrounding.instances()
        _fgs = []
        for fg in fgs:
            function_design = fg.typeFD
            for qa in function_design.hasQAestimation:
                if str(qa.isQAtype) == str(qa_key):
                    _fgs.append(fg)
        return _fgs

    # update QA value based on incoming diagnostic
    def update_qa(self, diagnostic_status):
        qa_type = self.get_qa_type(diagnostic_status.values[0].key)
        if qa_type is not None:
            fgs = self.get_function_groudings_require_qa(qa_type)
            value = float(diagnostic_status.values[0].value)
            with self.ontology_lock:
                measured_qa = update_measured_qa_value(
                    qa_type, value, self.tomasys, self.onto)
                for fg in fgs:
                    update_fg_measured_qa(fg, measured_qa)
            return_value = True
        else:
            return_value = False
        return return_value


    # For debugging purposes: saves state of the KB in an ontology file
    # TODO move to library
    # TODO save file in a temp location
    def save_ontology_exit(self, signal, frame):
        self.onto.save(file="error.owl", format="rdfxml")
        sys.exit(0)

    def handle_updatable_objectives(self, obj_in_error):
        if obj_in_error.o_status == "UPDATABLE":
            self.logger.info(
                ">> UPDATABLE objective - Try to clear Components status")
            for comp_inst in list(
                    self.tomasys.ComponentState.instances()):
                if comp_inst.c_status == "RECOVERED":
                    self.logger.info(
                        "Component {0} Status {1} - Set to None".format(
                            comp_inst.name, comp_inst.c_status))
                    comp_inst.c_status = None

    # selects configurations requested by the user
    def select_requested_configurations(self):
        objectives = self.search_objectives()
        requested_configurations = dict()
        for objective in objectives:
            function = objective.typeF.name
            if function in self.requested_configurations:
                requested_configurations[objective] = \
                    self.requested_configurations[function]
        self.requested_configurations = dict()
        return requested_configurations

    # find best fds for all objectives
    def select_desired_configuration(self, obj_in_error):
        self.logger.info(" >> Reasoner searches an FD ")
        desired_configuration = obtain_best_function_design(
            obj_in_error, self.tomasys)

        if desired_configuration is None:
            self.logger.warning(
                "No FD found to solve Objective {} ".format(obj_in_error.name))
        return desired_configuration

    def analyze(self):
        """
        Perform Analyze step of the MAPE-K loop

        :return: Adaptable Objectives
        :rtype: list[str]
        """
        self.logger.info(
            '>> Started MAPE-K ** Analysis (ontological reasoning) **')

        if self.kb_interface.has_objective() is False:
            self.kb_interface.print_ontology_status()
            return list()

        # Perform reasoning to infer new facts
        if self.kb_interface.perform_reasoning() is False:
            self.logger.error('>> Reasoning error')
            self.onto.save(
                file="error_reasoning.owl", format="rdfxml")

        # PRINT system status
        self.kb_interface.print_ontology_status()
        return self.kb_interface.get_objectives_in_error()

    # MAPE-K: Plan step
    def plan(self, objectives_in_error):
        """
        Perform Plan step of the MAPE-K loop

        :return: Selected configurations
        :rtype: dict[str, str]
        """
        if objectives_in_error == []:
            return dict()

        self.logger.info('  >> Started MAPE-K ** PLAN adaptation **')

        desired_configurations = self.select_requested_configurations()
        for obj_in_error in objectives_in_error:
            self.handle_updatable_objectives(obj_in_error)

            if obj_in_error not in desired_configurations:
                desired_config = self.select_desired_configuration(
                    obj_in_error)
                if desired_config is not None:
                    desired_configurations[obj_in_error] = desired_config
        return desired_configurations

    # MAPE-K: Execute step
    def execute(self, desired_configurations):
        if self.has_objective() is False or desired_configurations == dict():
            return

        self.logger.info('  >> Started MAPE-K ** EXECUTION **')
        for objective in desired_configurations:
            self.set_new_grounding(
                desired_configurations[objective], objective)
