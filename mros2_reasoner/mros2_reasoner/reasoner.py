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

from typing import Tuple


class Reasoner:

    def __init__(self, ontology_file):

        # owl model with the tomasys ontology
        self.tomasys = read_ontology_file(ontology_file)

        # This Lock is used to ensure safety of tQAvalues
        self.ontology_lock = Lock()

        # Dictionary with request configurations for functions
        self.requested_configurations = dict()

        self.logger = logging
        signal.signal(signal.SIGINT, self.save_ontology_exit)

    def remove_objective(self, objective_name: str) -> bool:
        # Checks if there are previously defined objectives.
        with self.ontology_lock:
            old_objective = self.tomasys.search_one(
                iri="*{}".format(objective_name), is_a=self.tomasys.Objective)
            if old_objective:
                old_fg_instance = self.tomasys.search_one(
                    solvesO=old_objective)
                destroy_entity(old_fg_instance)
                destroy_entity(old_objective)
                return True
            return False

    def search_objectives(self):
        with self.ontology_lock:
            objectives = self.tomasys.search(type=self.tomasys.Objective)
        return objectives

    def has_objective(self):
        objectives = self.search_objectives()
        has_objective = False
        if objectives != []:
            has_objective = True
        return has_objective

    def get_objective_from_objective_id(self, objective_id: str):
        objectives = self.search_objectives()
        if objectives == []:
            return None
        for objective in objectives:
            if str(objective.name) == str(objective_id):
                return objective
        return None

    def get_function_name_from_objective_id(self, objective_id: str) -> str:
        objective = self.get_objective_from_objective_id(objective_id)
        if objective is None:
            return None
        return str(objective.typeF.name)

    def get_objectives_in_error(self) -> Tuple[list[str], list[str]]:
        return get_objectives_in_error(self.search_objectives())

    def get_new_tomasys_objective(self, objective_name, iri_seed):
        """ Creates Objective individual in the KB given a desired name and a
        string seed for the Function name
        """
        with self.ontology_lock:
            objective = self.tomasys.Objective(
                str(objective_name),
                namespace=self.tomasys,
                typeF=self.tomasys.search_one(
                    iri=str(iri_seed)))
            return objective

    def get_new_tomasys_nfr(self, qa_value_name, nfr_key, nfr_value):
        """Creates QAvalue individual in the KB given a desired name and a
        string seed for the QAtype name and the value
        """
        with self.ontology_lock:
            # TODO: this search is not optimal, the search + loop can be
            # substituted by a single search
            qa_type = self.get_qa_type(nfr_key)
            new_nfr = self.tomasys.QAvalue(
                str(qa_value_name),
                namespace=self.tomasys,
                isQAtype=qa_type,
                hasValue=nfr_value)

            return new_nfr

    def append_nfr_to_objective(self, objective, nfr):
        with self.ontology_lock:
            objective.hasNFR.append(nfr)
        return objective

    def update_objective_status(self, objective, status):
        with self.ontology_lock:
            objective.o_status = status
        return objective

    def get_fg_solves_objective(self, objective):
        with self.ontology_lock:
            return self.tomasys.search_one(solvesO=objective)

    def set_new_grounding(self, fd_name: str, obj_name: str) -> str | None:
        """Given a string fd_name with the name of a FunctionDesign and an
        objective, removes the previous fg for the objective and ground a new
        fg of typeF fd
        """
        with self.ontology_lock:
            remove_objective_grounding(obj_name, self.tomasys)
            fd = self.tomasys.search_one(
                iri="*{}".format(fd_name),
                is_a=self.tomasys.FunctionDesign)
            if fd:
                # with self.ontology_lock:
                ground_fd(fd_name, obj_name, self.tomasys)
                reset_objective_status(obj_name, self.tomasys)
                return fd_name
            return None

    # the DiagnosticStatus message process contains, per field
    # - message: "binding_error"
    # - name: name of the fg reported, as named in the OWL file
    # - level: values 0 and 1 are mapped to nothing, values 2 or 3 are mapped
    # to fg.status="INTERNAL_ERROR"
    def update_binding(self, diagnostic_status):
        fg = self.tomasys.search_one(iri="*{}".format(diagnostic_status.name))
        if fg is None:
            return -1
        if diagnostic_status.level > 1:
            with self.ontology_lock:
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
        with self.ontology_lock:
            component_type = self.tomasys.search_one(
                iri="*{}".format(diagnostic_status.values[0].key))
            if component_type is not None:
                value = diagnostic_status.values[0].value
                reset_fd_realisability(
                    self.tomasys,
                    diagnostic_status.values[0].key)
                component_type.c_status = value
                return_value = 1
            else:
                return_value = 0
            return return_value

    def get_qa_type(self, key):
        # TODO: this search is not optimal, the search + loop can be
        # substituted by a single search or instances()
        qa_types = self.tomasys.search(type=self.tomasys.QualityAttributeType)
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
            if function_design is not None:
                for qa in function_design.hasQAestimation:
                    if str(qa.isQAtype) == str(qa_key):
                        _fgs.append(fg)
        return _fgs

    # update QA value based on incoming diagnostic
    def update_qa(self, diagnostic_status):
        with self.ontology_lock:
            qa_type = self.get_qa_type(diagnostic_status.values[0].key)
            if qa_type is not None:
                fgs = self.get_function_groudings_require_qa(qa_type)
                value = float(diagnostic_status.values[0].value)
                measured_qa = update_measured_qa_value(
                    qa_type, value, self.tomasys)
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
        with self.ontology_lock:
            with self.tomasys:
                try:
                    sync_reasoner_pellet(
                        infer_property_values=True,
                        infer_data_property_values=True,
                        debug=0
                    )
                except Exception as err:
                    self.logger.error(
                        "Error in perform_reasoning: {0}".format(err))
                    return False
                    # raise err
        return True

    # For debugging purposes: saves state of the KB in an ontology file
    # TODO move to library
    # TODO save file in a temp location
    def save_ontology_exit(self, signal, frame):
        self.tomasys.save(file="error.owl", format="rdfxml")
        sys.exit(0)

    def handle_updatable_objectives(self, obj: str) -> None:
        obj_in_error = self.get_objective_from_objective_id(obj)
        with self.ontology_lock:
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

    # find best fds for all objectives
    def select_desired_configuration(self, obj: str) -> dict[str, str]:
        obj_in_error = self.get_objective_from_objective_id(obj)
        with self.ontology_lock:
            self.logger.info(" >> Reasoner searches an FD ")
            desired_configuration = obtain_best_function_design(
                obj_in_error, self.tomasys)

            if desired_configuration is None:
                self.logger.warning(
                    "No FD found to solve Objective {} ".format(
                        obj_in_error.name))
            return desired_configuration

    # MAPE-K: Analyze step
    def analyze(self) -> list[str]:
        # PRINT system status
        with self.ontology_lock:
            print_ontology_status(self.tomasys)

        objectives_in_error = []
        if self.has_objective() is False:
            return objectives_in_error

        self.logger.info(
            '>> Started MAPE-K ** Analysis (ontological reasoning) **')

        # EXEC REASONING to update ontology with inferences
        if self.perform_reasoning() is False:
            with self.ontology_lock:
                self.logger.error('>> Reasoning error')
                self.tomasys.save(
                    file="error_reasoning.owl", format="rdfxml")
                return objectives_in_error

        # EVALUATE functional hierarchy (objectives statuses) (MAPE - Analysis)
        objectives_in_error, o_status = self.get_objectives_in_error()
        if objectives_in_error == []:
            self.logger.info(
                ">> No Objectives in ERROR: no adaptation is needed")
            return objectives_in_error

        for obj_in_error, status in zip(objectives_in_error, o_status):
            self.logger.warning(
                "Objective {0} in status: {1}".format(
                    obj_in_error, status))
        return objectives_in_error

    # MAPE-K: Plan step
    def plan(self, objectives_in_error: list[str]) -> dict[str, str]:
        if self.has_objective() is False or objectives_in_error == []:
            return dict()

        self.logger.info('  >> Started MAPE-K ** PLAN adaptation **')
        desired_configurations = dict()
        for obj_in_error in objectives_in_error:
            self.handle_updatable_objectives(obj_in_error)

            desired_config = self.select_desired_configuration(
                obj_in_error)
            if desired_config is not None:
                desired_configurations[obj_in_error] = desired_config
        return desired_configurations

    # MAPE-K: Execute step
    def execute(self, desired_configurations: dict[str, str]):
        if self.has_objective() is False or desired_configurations == dict():
            return

        self.logger.info('  >> Started MAPE-K ** EXECUTION **')
        for objective in desired_configurations:
            if self.get_objective_from_objective_id(objective) is not None:
                self.set_new_grounding(
                    desired_configurations[objective], objective)
