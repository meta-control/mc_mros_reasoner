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

import logging
from mros2_reasoner.tomasys_interface import TOMASysInterface
from owlready2 import destroy_entity
from owlready2 import get_ontology
from owlready2 import sync_reasoner_pellet
from threading import Lock


def read_ontology_file(ontology_file_array):
    """
    Read ontology from file.

    Parameters
    ----------
    ontology_file_array: string|array of string
        path of the ontology to be loaded.

    Returns
    -------
    ontology: variable with the loaded ontology.

    """
    try:
        if (isinstance(ontology_file_array, str)):
            ontology_file_array = [ontology_file_array]
        ontology_obj = None
        for ontology_file in ontology_file_array:
            if ontology_file is not None:
                ontology = get_ontology(ontology_file).load()
                if ontology is not None:
                    logging.info('Loaded ontology: ' + str(ontology_file))
                else:
                    logging.error(
                        'Failed to load ontology from: ' +
                        str(ontology_file))
                    return None
                if ontology_obj:
                    ontology_obj.imported_ontologies.append(ontology)
                else:
                    ontology_obj = ontology
        return ontology_obj
    except Exception as e:
        logging.exception(
            ' When reading the ontology {0}, returned exception {1}'.format(
                ontology_file_array, e)
        )
        raise e


class OwlReady2TOMASys(TOMASysInterface):
    """! OwlReady2 interface for TOMASys."""

    def __init__(self):
        super().__init__()
        self.knowledge_base = None
        self.knowledge_base_lock = Lock()
        self.objectives_error_status = [
            'UNGROUNDED',
            'UPDATABLE',
            'IN_ERROR_FR',
            'IN_ERROR_NFR',
            'IN_ERROR_COMPONENT'
        ]

    def load_ontology_file(self, file_array):
        self.knowledge_base = read_ontology_file(file_array)

    def get_class_instances(self, class_name):
        if self.knowledge_base is None:
            logging.warning('Knowledge Base is not loaded!')
            return list()
        return self.knowledge_base[class_name].instances()

    def get_class_individual(self, class_name, individual_name):
        if self.knowledge_base is None:
            logging.warning('Knowledge Base is not loaded!')
            return None
        return self.knowledge_base.search_one(
            is_a=self.knowledge_base[class_name],
            iri='*{}'.format(individual_name)
        )

    def create_instance(self, class_name, instance_name, property_dict={}):
        if self.knowledge_base is None:
            logging.warning('Knowledge Base is not loaded!')
            return None
        class_ = getattr(self.knowledge_base, class_name)
        # Is the lock needed?
        with self.knowledge_base_lock:
            instance = class_(
                name=instance_name, namespace=self.knowledge_base)
            for property in property_dict:
                setattr(instance, property, property_dict[property])
            return instance

    def remove_instance(self, class_name, instance_name):
        individual = self.get_class_individual(class_name, instance_name)
        if individual is None:
            logging.warning(
                'Instance {0} of class {1} is not in the knowledge base!'
                .format(instance_name, class_name))
            return False
        self.destroy_entity(individual)
        return True

    def destroy_entity(self, entity):
        # Is the lock needed?
        with self.knowledge_base_lock:
            return destroy_entity(entity)

    def remove_objectives(self):
        objectives = self.get_objectives()
        for obj in objectives:
            self.destroy_entity(obj)

    def get_objectives(self):
        return self.get_class_instances('Objective')

    def has_objective(self):
        objectives = self.get_objectives()
        has_objective = False
        if len(objectives) > 0:
            has_objective = True
        return has_objective

    def get_objectives_in_error(self):
        objectives = self.get_objectives()
        objectives_in_error = list()
        for o in objectives:
            if o.o_status in self.objectives_error_status:
                objectives_in_error.append(o)
        return objectives_in_error
    # def get_function_groundings(self):
    #     return self.get_class_instances('FunctionGrounding')

    def get_function_designs(self):
        return self.get_class_instances('FunctionDesign')

    # EXEC REASONING to update ontology with inferences
    # TODO CHECK: update reasoner facts, evaluate, retrieve action, publish
    # update reasoner facts
    def perform_reasoning(self):
        with self.knowledge_base_lock:
            with self.knowledge_base:
                try:
                    sync_reasoner_pellet(
                        infer_property_values=True,
                        infer_data_property_values=True)
                    return True
                except Exception as err:
                    self.logger.error(
                        "Error in perform_reasoning: {0}".format(err))
                    return False
                    # raise err

    def print_ontology_status(self):
        logging.info('\t\t\t >>> Ontology Status   <<<')

        components = [
            (c.name, c.c_status)
            for c in self.get_class_instances('ComponentState')]
        logging.info('\n\tComponent Status:\t{0}'.format(components))

        fgs = self.get_class_instances('FunctionGrounding')
        for fg in fgs:
            qas = [(qa.isQAtype.name, qa.hasValue) for qa in fg.hasQAvalue]
            logging.info(
                '\n\tFG: {0}  Status: {1}  Solves: {2}  FD: {3}  QAvalues: {4}'
                .format(
                    fg.name,
                    fg.fg_status,
                    fg.solvesO.name,
                    fg.typeFD.name,
                    qas
                )
            )

        objectives = self.get_class_instances('Objective')
        for objective in objectives:
            qas = [
                (nfr.isQAtype.name, nfr.hasValue) for nfr in objective.hasNFR]
            logging.info(
                '\n\tOBJECTIVE: {0}   Status: {1}   NFRs:  {2}'.format(
                    objective.name,
                    objective.o_status,
                    qas
                )
            )
        logging.info('\t\t\t >>>>>>>>>>>>> <<<<<<<<<<<')
