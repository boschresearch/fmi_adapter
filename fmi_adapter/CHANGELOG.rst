^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fmi_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.1.0 (2021-03-24)
------------------
* Adapted launch files to API changes.

2.0.0 (2021-03-24)
------------------
* Added function 'getValue' to return value of any given variable name
  Co-authored-by: Sebastian Zarnack <sebastian.zarnack@eas.iis.fraunhofer.de>
* Improved readability of unit tests by chrono literals.
* Replaced use of deprecated Duration ctor.
* Added virtual to lifecycle callbacks, as in interface.

0.1.8 (2020-05-14)
------------------
* Prepared for Foxy release.

0.1.7 (2020-01-30)
------------------
* Fixed sporadic exception in case of small external steps.
* Fixed fmuLocation argument for fmi2_import_instantiate.

0.1.6 (2019-11-05)
------------------
* Release for ROS 2 Eloquent.
* Changed build files for use of fmilibrary_vendor package.

0.1.5 (2019-05-24)
------------------
* Adapted to new Dashing features, including QoS, parameter declaration and node composition.

0.1.4 (2019-05-23)
------------------
* Fixed link to FMU-SDK.

0.1.3 (2019-02-01)
------------------
* Fixed install target location of shared library.
* Improved code snippets on use of FMIAdapter class in README.

0.1.2 (2019-01-25)
------------------
* Cleaned up dependency entries in package.xml.
* Added explicit target dependencies for parallel building.

0.1.1 (2019-01-23)
------------------
* Fixed missing testing and launch dependencies.

0.1.0 (2019-01-18)
------------------
* Initial version for ROS 2, ported from https://github.com/boschresearch/fmi_adapter/
