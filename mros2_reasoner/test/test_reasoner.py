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
from mros2_reasoner.reasoner import Reasoner

import pytest
import os
from pathlib import Path


@pytest.fixture()
def reasoner():
    path_to_pkg = Path(__file__).parents[1]
    path_test_data = path_to_pkg / 'test' / 'owl'

    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    ontology_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(path_test_data, 'test.owl')]
    yield Reasoner(ontology_files_array)


def test_reasoner_analyze(reasoner):
    expected_result = [
        reasoner.kb_interface.get_class_individual(
            'Objective', 'o_fake_infer_component_in_error'),
        reasoner.kb_interface.get_class_individual(
            'Objective', 'o_fake_infer_in_error_nfr'),
    ]
    adaptable_objectives = reasoner.analyze()
    assert all(r in adaptable_objectives for r in expected_result)
