^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fmi_adapter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
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
