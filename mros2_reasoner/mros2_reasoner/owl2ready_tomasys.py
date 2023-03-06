import logging
from mros2_reasoner.tomasys_interface import TOMASysInterface
from owlready2 import get_ontology


def read_ontology_file(ontology_file_array):
    """! Reads ontology from file
        Args:
            ontology_file_array (string|array of string): Full path to
            the ontology to be loaded.
        Returns:
    """
    try:
        if (type(ontology_file_array) == str):
            ontology_file_array = [ontology_file_array]
        ontology_obj = None
        for ontology_file in ontology_file_array:
            if ontology_file is not None:
                ontology = get_ontology(ontology_file).load()
                if ontology is not None:
                    logging.info("Loaded ontology: " + str(ontology_file))
                else:
                    logging.error(
                        "Failed to load ontology from: " +
                        str(ontology_file))
                    return None
                if ontology_obj:
                    ontology_obj.imported_ontologies.append(ontology)
                else:
                    ontology_obj = ontology
        return ontology_obj
    except Exception as e:
        logging.exception(
            "When reading the ontology {0}, returned exception {0}".format(
                ontology_file_array, e)
        )
        raise e


class Owl2ReadyTOMASys(TOMASysInterface):
    """! Owl2Ready interface for TOMASys
    """
    def __init__(self):
        super().__init__()

    def load_ontology_file(self, file_array):
        self.knowledge_base = read_ontology_file(file_array)

    def get_instances(self, class_name):
        return self.knowledge_base[class_name].instances()
