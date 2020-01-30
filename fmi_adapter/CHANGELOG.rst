^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fmi_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.3 (2020-01-30)
------------------
* Fixed sporadic exception in case of small external steps.
* Updated to FMILibrary version 2.1 and to new location of FMILibrary sources.
* Fixed fmuLocation argument for fmi2_import_instantiate.

1.0.2 (2018-10-12)
------------------
* Added parameter to configure update period of fmi_adapter node.
* Introduced functions for single step and replaced calcUntil with doStepsUntil.
* Enable automatic use of default experiment step-size from FMU in FMIAdapter ctor.
* Added function to query default experiment step-size given in the FMU.
* Added step_size parameter and function to query whether the FMU supports a variable communication step size.

1.0.1 (2018-07-16)
------------------
* Throwing runtime_error in case of failed fmi2 function call.

1.0.0 (2018-07-13)
------------------
* Initial version.
