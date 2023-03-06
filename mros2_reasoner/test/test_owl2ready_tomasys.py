from ament_index_python.packages import get_package_share_directory
from mros2_reasoner.owl2ready_tomasys import Owl2ReadyTOMASys
from mros2_reasoner.owl2ready_tomasys import read_ontology_file
from pathlib import Path
import pytest
import os


def test_load_file_str():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    tomasys_path = os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl')

    assert read_ontology_file(tomasys_path) is not None


def test_load_file_array():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    ontology_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mock.owl')]

    ontology = read_ontology_file(ontology_files_array)
    assert ontology is not None and len(ontology.imported_ontologies) == 2


def test_load_file_nonexistent():
    path = "/tmp/mros_nonexistent"
    with pytest.raises(Exception) as e:
        read_ontology_file(path)


def test_load_file_empty():
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

    kb_interface = Owl2ReadyTOMASys()
    kb_interface.load_ontology_file(ontology_files_array)
    return kb_interface


def test_load_file_class():
    kb_interface = load_mock_ontology()
    assert kb_interface.knowledge_base is not None


def test_get_instances():
    kb_interface = load_mock_ontology()
    class_name = "Function"
    instances = kb_interface.get_instances(class_name)
    correct_instance_type = True
    for instance in instances:
        correct_instance_type = correct_instance_type and (
         instance.is_instance_of[0] == kb_interface.knowledge_base[class_name])

    assert correct_instance_type is True and len(list(instances)) > 0
