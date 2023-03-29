
# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).
## [1.0.1] - 29-03-2023

### Fixed

1. Minor bug fix, commit [9e687f5](https://github.com/meta-control/mc_mros_reasoner/commit/9e687f5e68a99cc52ecb83f1a7a046d36f35029b)

## [1.0.0] - 07-02-2023

MROS main version is now ROS2-based, and tested with ROS humble.

### Added

1. Add CI, for now it is only testing if package can be built
  * [Issue 136](https://github.com/meta-control/mc_mros_reasoner/issues/136)


2. Add support for multiple objectives
  * [Issue 146](https://github.com/meta-control/mc_mros_reasoner/issues/146)

### Changed

1. Reorganized branches: master is now ROS2-based
  * [Issue 135](https://github.com/meta-control/mc_mros_reasoner/issues/135)


2. Separation of system modes from MROS
  * [Issue 138](https://github.com/meta-control/mc_mros_reasoner/issues/138)
  * [Issue 139](https://github.com/meta-control/mc_mros_reasoner/issues/139)
  * [Issue 150](https://github.com/meta-control/mc_mros_reasoner/issues/150)
  * [Pull request 160](https://github.com/meta-control/mc_mros_reasoner/pull/160)


3. Objectives can be in `IN_ERROR_FR` or `IN_ERROR_NFR`, and not all Function Designs in error are blocked
  * [Issue 141](https://github.com/meta-control/mc_mros_reasoner/issues/141)
  * [Issue 143](https://github.com/meta-control/mc_mros_reasoner/issues/143)
  * [Issue 144](https://github.com/meta-control/mc_mros_reasoner/issues/144)
  * MROS ontology commits: [dd139f0](https://github.com/meta-control/mros_ontology/commit/dd139f07854b52f3298d23c0a1232cce28cd538a) and [a020191](https://github.com/meta-control/mc_mdl_tomasys/commit/a020191e7e7c52d4f5aa3e2240fca2ea269bf940)


4. Renamed objective action name, and change mode service name
  * [Issue 161](https://github.com/meta-control/mc_mros_reasoner/issues/161)


5. Method used to read the ontology files was moved to the tomasys lib


6. RosReasoner inherits from Reasoner


7. In the meetNFR function, it returns without doing anything if no requirement is hardcoded   


8. Added mros2_mock package with mock code to test MROS2


9. Updates to package.xml


10. Formatting following pep8


11. Refactoring in general


12. Several changes to logging

### Fixed

1. [Fix issue #130](https://github.com/meta-control/mc_mros_reasoner/issues/130): Fix system modes' service import
  * [Pull request 132](https://github.com/meta-control/mc_mros_reasoner/pull/132)


2. Several minor fixes
