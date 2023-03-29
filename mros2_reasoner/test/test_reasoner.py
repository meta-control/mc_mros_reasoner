from ament_index_python.packages import get_package_share_directory
from mros2_reasoner.reasoner import Reasoner

import pytest
import os


def create_reasoner():
    pkg_mc_mdl_tomasys_path = get_package_share_directory('mc_mdl_tomasys')
    pkg_mros_ontology_path = get_package_share_directory('mros_ontology')

    ontology_files_array = [
        os.path.join(pkg_mc_mdl_tomasys_path, 'owl', 'tomasys.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mros.owl'),
        os.path.join(pkg_mros_ontology_path, 'owl', 'mock.owl')]
    return Reasoner(ontology_files_array)


def test_reasoner_analyze():
    reasoner = create_reasoner()
    reasoner.analyze()
    # assert False
