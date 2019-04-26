General information about this repository, including legal information, build instructions and known issues/limitations, are given in [README.md](../README.md) in the repository root.


# The fmi_adapter_examples package

This [ROS 2](http://www.ros.org/) package provides few examples for the use of the fmi_adapter package. It contains two FMU files DampedPendulum.fmu and TransportDelay.fmu (both created with the [FMU SDK](https://www.qtronic.de/en/fmu-sdk/)) and corresponding launch files. Furthermore, it includes a model of a damped pendulum in the [Modelica language](https://www.modelica.org/) to create your own FMU.


## Running the provided sample FMUs

Use `ros2 launch fmi_adapter_examples simple_damped_pendulum.launch.py` to simulate a damped pendulum ([share/DampedPendulum.fmu](share/DampedPendulum.fmu)) with a length of 1m. The pendulum's angle is published at topic /a with the default rate of 100Hz. The step size of the FMU's solver is 1ms.

To print the angle data on another console, invoke `ros2 topic echo /a`.

[damped_pendulum_with_transport_delay.launch.py](launch/damped_pendulum_with_transport_delay.launch) starts two nodes named */example/damped_pendulum* and */example/transport_delay*. The first one simulates [share/DampedPendulum.fmu](share/DampedPendulum.fmu), where the length parameter is set to 25m by the launch file. The second node runs [share/TransportDelay.fmu](share/TransportDelay.fmu), where the delay parameter is set to 2.33s. The input subscription of the transport delay is remapped to the pendulum's angle topic and the delayed angle is published at */example/y*.


## Create and simulate your own DampedPendulum.fmu

There are several modeling tools that support the Modelica language and provide FMU export. Examples are [Dymola](http://www.3ds.com/products-services/catia/products/dymola), [JModelica](https://jmodelica.org/), and [OpenModelica](https://www.openmodelica.org/).

In the following, we explain the process by the example of OpenModelica, which has been also used to create the model of the damped pendulum at [share/DampedPendulum.mo](share/DampedPendulum.mo).

*   Download and install OpenModelica for Linux as described in [https://openmodelica.org/download/download-linux](https://openmodelica.org/download/download-linux).
*   Launch `OMEdit` and load the [share/DampedPendulum.mo](share/DampedPendulum.mo) model file.
*   Click on the DampedPendulum model in the project tree on the left.

![Screenshot of the DampedPendulum model in OMEdit V1.12.0](doc/damped_pendulum_in_OMEdit.png)

*   Navigate to Tools -> Options -> FMI and ensure that `Version=2.0`, `Type=Co-Simulation` and `Platforms=Dynamic` is selected.
*   Then click FMI -> Export FMU.
*   The path of the resulting FMU file is shown in the message browser at the bottom of the window, typically `/tmp/OpenModelica_[user]/OMEdit/DampedPendulum.fmu`.

Now, you are prepared for simulating the FMU using the fmi_adapter package.

*   Use the generic launch file of the package
    `ros2 launch fmi_adapter fmi_adapter_node.launch.py fmu_path:=/tmp/OpenModelica_[user]/OMEdit/DampedPendulum.fmu`

*   You may print the pendulum's angle to the console by
    `ros2 topic echo /revolute1_angle`

Please see the [README.md of the fmi_adapter package](../fmi_adapter/README.md) for how to load and run an FMU inside an application-specific ROS node or library.


**Note on bug with mmc_mk_modelica_array in OpenModelica 1.12.0:** If fmi_adapter crashes with the error message `undefined symbol: mmc_mk_modelica_array`, please patch the files

*   /usr/include/omc/c/meta/meta_modelica.h
*   /usr/include/omc/c/meta/meta_modelica_data.h

according to [https://github.com/OpenModelica/OMCompiler/pull/2397/files](https://github.com/OpenModelica/OMCompiler/pull/2397/files) and export the FMU again. Details on this bug are given in [https://trac.openmodelica.org/OpenModelica/ticket/4899](https://trac.openmodelica.org/OpenModelica/ticket/4899).
