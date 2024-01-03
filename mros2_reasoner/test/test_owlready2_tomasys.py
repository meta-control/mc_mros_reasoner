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
    assert correct_instance_type is True and len(instances) > 0


def test_owlready2_get_objectives():
    kb_interface = load_mock_ontology()
    class_name = 'Objective'
    instance_name = 'test_objective'
    property_dict = {
        'typeF': kb_interface.get_class_individual('Function', 'f_mock')
    }
    kb_interface.create_instance(class_name, instance_name, property_dict)

    instances = kb_interface.get_objectives()
    correct_instance_type = True
    for instance in instances:
        correct_instance_type = correct_instance_type and (
         instance.is_instance_of[0] == kb_interface.knowledge_base[class_name])
    assert correct_instance_type is True and len(instances) > 0


def test_owlready2_get_objectives_in_error():
    kb_interface = load_mock_ontology()
    property_dict = {
        'o_status': 'UNGROUNDED',
    }
    obj = kb_interface.create_instance('Objective', 'obj_error', property_dict)
    objectives_in_error = kb_interface.get_objectives_in_error()
    assert obj in objectives_in_error


def test_owlready2_has_objective():
    kb_interface = load_mock_ontology()
    kb_interface.remove_objectives()

    objective_false = kb_interface.has_objective()

    kb_interface.create_instance('Objective', 'test_objective')
    objective_true = kb_interface.has_objective()

    assert (objective_false is False) and (objective_true is True)


def test_owlready2_get_function_designs():
    kb_interface = load_mock_ontology()
    class_name = "FunctionDesign"
    instances = kb_interface.get_function_designs()
    correct_instance_type = True
    for instance in instances:
        correct_instance_type = correct_instance_type and (
         instance.is_instance_of[0] == kb_interface.knowledge_base[class_name])
    assert correct_instance_type is True and len(instances) > 0


def test_owlready2_get_class_instances_none():
    kb_interface = OwlReady2TOMASys()
    class_name = "Function"
    instances = kb_interface.get_class_instances(class_name)
    assert len(instances) == 0


def test_owlready2_get_class_individual():
    kb_interface = load_mock_ontology()
    class_name = "Function"
    individual = kb_interface.get_class_individual(class_name, 'f_mock')
    correct_cls = individual.is_a[0] == kb_interface.knowledge_base[class_name]
    assert individual.name == 'f_mock' and correct_cls


def test_owlready2_get_class_individual_none():
    kb_interface = OwlReady2TOMASys()
    class_name = "Function"
    individual = kb_interface.get_class_individual(class_name, 'f_mock')
    assert individual is None


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


def test_owlready2_create_instances_none():
    kb_interface = OwlReady2TOMASys()
    class_name = 'Objective'
    instance_name = 'test_objective'
    property_dict = dict()
    instance = kb_interface.create_instance(
        class_name, instance_name, property_dict)

    assert instance is None


def test_owlready2_remove_instance():
    kb_interface = load_mock_ontology()

    kb_interface.create_instance('Function', 'test_function_remove')
    f = kb_interface.get_class_individual('Function', 'test_function_remove')
    removed = kb_interface.remove_instance('Function', 'test_function_remove')
    f2 = kb_interface.get_class_individual('Function', 'test_function_remove')

    assert (f is not None) and removed and (f2 is None)


def test_owlread2_perform_reasoning():
    kb_interface = load_mock_ontology()
    reasoning = kb_interface.perform_reasoning()
    assert reasoning is True


def test_owlready2_print_ontology():
    kb_interface = load_mock_ontology()
    kb_interface.print_ontology_status()
