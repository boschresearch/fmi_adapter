[![License](https://img.shields.io/badge/License-Apache%202-blue.svg)](LICENSE)
[![Build status](http://build.ros2.org/job/Fdev__fmi_adapter__ubuntu_focal_amd64/badge/icon?subject=Build%20farm%3A%20Foxy)](http://build.ros2.org/job/Fdev__fmi_adapter__ubuntu_focal_amd64/)
[![Build status](http://build.ros2.org/job/Hdev__fmi_adapter__ubuntu_jammy_amd64/badge/icon?subject=Build%20farm%3A%20Humble)](http://build.ros2.org/job/Hdev__fmi_adapter__ubuntu_jammy_amd64/)
[![Build status](http://build.ros2.org/job/Rdev__fmi_adapter__ubuntu_jammy_amd64/badge/icon?subject=Build%20farm%3A%20Rolling)](http://build.ros2.org/job/Rdev__fmi_adapter__ubuntu_jammy_amd64/)
[![Build status](https://github.com/boschresearch/fmi_adapter/workflows/Build%20action%3A%20Foxy%2C%20Humble%2C%20Rolling/badge.svg)](https://github.com/boschresearch/fmi_adapter/actions)
[![Code coverage](https://codecov.io/gh/boschresearch/fmi_adapter/branch/rolling/graph/badge.svg)](https://codecov.io/gh/boschresearch/fmi_adapter)

# The fmi_adapter repository

This repository provides the fmi_adapter package for wrapping *functional mockup units (FMUs)* for co-simulation of physical models into ROS 2 nodes, i.e. for the version ROS 2. The implementation for the first generation of ROS can be found in the [melodic_and_noetic branch](https://github.com/boschresearch/fmi_adapter/tree/melodic_and_noetic).


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

fmi_adapter is open-sourced under the Apache-2.0 license. See the [LICENSE](LICENSE) file for details.

For a list of other open source components included in fmi_adapter, see the file [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Quality assurance

The colcon_test tool is used for quality assurances, which includes cpplint, uncrustify, flake8, xmllint and various other tools.

Unit tests based on [gtest](https://github.com/google/googletest) are located in the [fmi_adapter/test](fmi_adapter/test) folder. The unit tests use an FMU created with the [FMU SDK](https://www.qtronic.de/en/fmu-sdk/) by QTronic GmbH, cf. [3rd-party-licenses.txt](3rd-party-licenses.txt).


## Known issues/limitations

Please notice the following issues/limitations:

*   fmi_adapter only supports FMUs according to the FMI 2.0 standard.
*   fmi_adapter treats all inputs, outputs and parameters of a given FMU as floating-point values (ROS message std_msgs::msg::Float64, C++ type double, FMI type fmi2fmi2_real_t).
*   A possible end time specified in an FMU is not considered, i.e., the FMU is being evaluated constantly until the corresponding ROS node is shutdown.


## Papers

If you want to cite this repository/package, please cite the following book chapter ([PDF available at Springer Link](https://doi.org/10.1007/978-3-030-45956-7_7)) instead:

Ralph Lange, Silvio Traversaro, Oliver Lenord, and Christian Bertsch: Integrating the Functional Mock-Up Interface with ROS and Gazebo. In: _Anis Koubaa (ed.) Robot Operating System (ROS): The Complete Reference (Volume 5)_, Springer, pp. 187–231, 2021.

```bibtex
@INBOOK{Lange_et_al_2021_Integrating_the_FMI_with_ROS_and_Gazebo,
  author = {Ralph Lange and Silvio Traversaro and Oliver Lenord and Christian Bertsch},
  title = {Integrating the Functional Mock-Up Interface with ROS and Gazebo},
  editor = {Anis Koubaa},
  booktitle = {Robot Operating System (ROS): The Complete Reference (Volume 5)},
  year = {2021},
  publisher = {Springer},
  pages = {187--231},
  doi = {10.1007/978-3-030-45956-7_7}
}
```
