from ament_index_python.packages import get_package_share_directory
from mros2_reasoner.owlready2_tomasys import OwlReady2TOMASys
from mros2_reasoner.owlready2_tomasys import read_ontology_file
from pathlib import Path
import pytest
import os


def test_owlready2_load_file_str():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    tomasys_path = os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl')

    assert read_ontology_file(tomasys_path) is not None


def test_owlready2_load_file_array():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    ontology_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mock.owl')]

    ontology = read_ontology_file(ontology_files_array)
    assert ontology is not None and len(ontology.imported_ontologies) == 2


def test_owlready2_load_file_nonexistent():
    path = "/tmp/mros_nonexistent"
    with pytest.raises(Exception) as e:
        read_ontology_file(path)


def test_owlready2_load_file_empty():
    path = '/tmp/mros_emptyfile.owl'
    Path(path).touch()
    assert read_ontology_file(path) is not None


def load_mock_ontology():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    ontology_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mock.owl')]

    kb_interface = OwlReady2TOMASys()
    kb_interface.load_ontology_file(ontology_files_array)
    return kb_interface


def test_owlready2_load_file_class():
    kb_interface = load_mock_ontology()
    assert kb_interface.knowledge_base is not None


def test_owlready2_get_class_instances():
    kb_interface = load_mock_ontology()
    class_name = "Function"
    instances = kb_interface.get_class_instances(class_name)
    correct_instance_type = True
    for instance in instances:
        correct_instance_type = correct_instance_type and (
         instance.is_instance_of[0] == kb_interface.knowledge_base[class_name])

    assert correct_instance_type is True and len(list(instances)) > 0


def test_owlready2_get_class_individual():
    kb_interface = load_mock_ontology()
    class_name = "Function"
    individual = kb_interface.get_class_individual(class_name, 'f_mock')
    correct_cls = individual.is_a[0] == kb_interface.knowledge_base[class_name]
    assert individual.name == 'f_mock' and correct_cls


def test_owlready2_create_instances():
    kb_interface = load_mock_ontology()
    class_name = 'Objective'
    instance_name = 'test_objective'
    property_dict = {
        'typeF': kb_interface.get_class_individual('Function', 'f_mock')
    }
    kb_interface.create_instance(class_name, instance_name, property_dict)
    test_obj = kb_interface.get_class_individual('Objective', 'test_objective')

    assert test_obj.name == 'test_objective' and \
        test_obj.typeF.is_a[0] == kb_interface.knowledge_base['Function'] and \
        test_obj.typeF.name == 'f_mock'
