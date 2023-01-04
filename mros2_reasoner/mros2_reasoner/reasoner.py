#!/usr/bin/env python
###########################################
#
# authors:    M.A.GarzonOviedo@tudelft.nl
#             c.h.corbato@tudelft.nl
##########################################

import signal
import sys
from threading import Lock

from mros2_reasoner.tomasys import get_objectives_in_error
from mros2_reasoner.tomasys import ground_fd
from mros2_reasoner.tomasys import obtain_best_function_design
from mros2_reasoner.tomasys import print_ontology_status
from mros2_reasoner.tomasys import read_ontology_file
from mros2_reasoner.tomasys import remove_objective_grounding
from mros2_reasoner.tomasys import reset_fd_realisability
from mros2_reasoner.tomasys import reset_objective_status
from mros2_reasoner.tomasys import update_fg_measured_qa
from mros2_reasoner.tomasys import update_measured_qa_value

from owlready2 import destroy_entity
from owlready2 import sync_reasoner_pellet

import logging


class Reasoner:

    def __init__(self, tomasys_file, model_file):

        # owl model with the tomasys ontology
        self.tomasys = read_ontology_file(tomasys_file)
        # application model as individuals of tomasys classes
        self.onto = read_ontology_file(model_file)

        # This Lock is used to ensure safety of tQAvalues
        self.ontology_lock = Lock()

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

    def search_objectives(self):
        objectives = self.onto.search(type=self.tomasys.Objective)
        return objectives

    def has_objective(self):
        objectives = self.search_objectives()
        has_objective = False
        if objectives != []:
            has_objective = True
        return has_objective

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

    def get_new_tomasys_objective(self, objective_name, iri_seed):
        """ Creates Objective individual in the KB given a desired name and a
        string seed for the Function name
        """
        objective = self.tomasys.Objective(
            str(objective_name),
            namespace=self.onto,
            typeF=self.onto.search_one(
                iri=str(iri_seed)))
        return objective

    def get_new_tomasys_nfr(self, qa_value_name, nfr_key, nfr_value):
        """Creates QAvalue individual in the KB given a desired name and a
        string seed for the QAtype name and the value
        """

        # TODO: this search is not optimal, the search + loop can be
        # substituted by a single search
        qa_type = self.get_qa_type(nfr_key)

        new_nfr = self.tomasys.QAvalue(
            str(qa_value_name),
            namespace=self.onto,
            isQAtype=qa_type,
            hasValue=nfr_value)

        return new_nfr

    def set_new_grounding(self, fd_name, objective):
        """Given a string fd_name with the name of a FunctionDesign and an
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

    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    def perform_reasoning(self):
        return_value = False
        with self.ontology_lock:
            with self.onto:
                try:
                    sync_reasoner_pellet(
                        infer_property_values=True,
                        infer_data_property_values=True)
                    return_value = True
                except Exception as err:
                    self.logger.error("{0}".format(err))
                    return False
                    # raise err

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

    # MAPE-K: Analyze step
    def analyze(self):
        # PRINT system status
        print_ontology_status(self.tomasys)

        objectives_in_error = []
        if self.has_objective() is False:
            return objectives_in_error

        self.logger.info(
            '>> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if self.perform_reasoning() is False:
            self.logger.error('>> Reasoning error')
            self.onto.save(
                file="error_reasoning.owl", format="rdfxml")
            return objectives_in_error

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_in_error = self.get_objectives_in_error()
        if objectives_in_error == []:
            self.logger.info(
                ">> No Objectives in ERROR: no adaptation is needed")
        else:
            for obj_in_error in objectives_in_error:
                self.logger.warning(
                    "Objective {0} in status: {1}".format(
                        obj_in_error.name, obj_in_error.o_status))
        return objectives_in_error

    # MAPE-K: Plan step
    def plan(self, objectives_in_error):
        if self.has_objective() is False or objectives_in_error == []:
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
