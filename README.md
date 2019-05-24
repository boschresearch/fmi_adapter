# The fmi_adapter_ros2 repository

[![Build Status](http://build.ros2.org/job/Ddev__fmi_adapter_ros2__ubuntu_bionic_amd64/badge/icon)](http://build.ros2.org/job/Ddev__fmi_adapter_ros2__ubuntu_bionic_amd64/)

This repository provides the fmi_adapter package for wrapping *functional mockup units (FMUs)* for co-simulation of physical models into ROS 2 nodes, i.e. for the version ROS 2. The original implementation for the first generation of ROS can be found at [github.com/boschresearch/fmi_adapter](https://github.com/boschresearch/fmi_adapter).


FMUs are defined in the [FMI standard](http://fmi-standard.org/) and can be created with a variety of modeling and simulation tools, including [Dymola](http://www.3ds.com/products-services/catia/products/dymola), [MATLAB/Simulink](https://www.mathworks.com/products/simulink.html), [OpenModelica](https://www.openmodelica.org/), [SimulationX](https://www.simulationx.de/), and [Wolfram System Modeler](http://www.wolfram.com/system-modeler/).

fmi_adapter provides a library with convenience functions based on common ROS types to load an FMU during runtime, retrieve the input, output, and parameter names, set timestamped input values, run the FMU's numeric solver, and query the resulting output.

In detail, this repository contains two ROS 2 packages:

*   [fmi_adapter](fmi_adapter/) provides a generic library and node for loading and running FMUs in ROS-based applications.
*   [fmi_adapter_examples](fmi_adapter_examples/) provides small examples for the use of fmi_adapter.

Technical information on the interfaces and use of these packages is given in the README.md files in the corresponding subfolders.


## Purpose of the project

The software is not ready for production use. It has neither been developed nor tested for a specific use case. However, the license conditions of the applicable Open Source licenses allow you to adapt the software to your needs. Before using it in a safety relevant setting, make sure that the software fulfills your requirements and adjust it according to any applicable safety standards (e.g. ISO 26262).


## Requirements, how to build, test, install, use, etc.

Clone the repository into a ROS workspace and build it using [colcon](https://colcon.readthedocs.io/).


## License

fmi_adapter_ros2 is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in fmi_adapter_ros2, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Quality assurance

The colcon_test tool is used for quality assurances, which includes cpplint, uncrustify, flake8, xmllint and various other tools.

Unit tests based on [gtest](https://github.com/google/googletest) are located in the [fmi_adapter/test](fmi_adapter/test) folder. The unit tests use an FMU created with the [FMU SDK](https://www.qtronic.de/en/fmu-sdk/) by QTronic GmbH, cf. [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Known issues/limitations

Please notice the following issues/limitations:

*   fmi_adapter only supports FMUs according to the FMI 2.0 standard.
*   fmi_adapter treats all inputs, outputs and parameters of a given FMU as floating-point values (ROS message std_msgs::msg::Float64, C++ type double, FMI type fmi2fmi2_real_t).
