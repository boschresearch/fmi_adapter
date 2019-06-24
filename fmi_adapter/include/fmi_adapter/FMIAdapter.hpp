// Copyright (c) 2019 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter_ros2.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef FMI_ADAPTER__FMIADAPTER_HPP_
#define FMI_ADAPTER__FMIADAPTER_HPP_

#include <cassert>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

struct fmi_xml_context_t;
typedef struct fmi_xml_context_t fmi_import_context_t;
struct fmi2_import_t;
struct fmi2_xml_variable_t;
typedef struct fmi2_xml_variable_t fmi2_import_variable_t;
struct jm_callbacks;

namespace fmi_adapter
{

/// An instance of this class wraps a FMU and allows to simulate it using the ROS time notion and
/// standard C++ types. In the background, the FMI Library (a BSD-licensed C library) is used for
/// interacting with the FMU. This class also provides concenvience functions to read parameters
/// and initial values from ROS parameters.
class FMIAdapter
{
public:
  /// This ctor creates an instance using the FMU from the given path. If the step-size argument
  /// is zero, the default experiment step-size given in the FMU is used.
  explicit FMIAdapter(
    rclcpp::Logger logger, const std::string & fmuPath,
    rclcpp::Duration stepSize = rclcpp::Duration(0),
    bool interpolateInput = true, const std::string & tmpPath = "");

  RCLCPP_DISABLE_COPY(FMIAdapter)

  virtual ~FMIAdapter();

  /// This helper function returns a ROSified version of the given variable name for use with ROS
  /// parameters. It simply replaces all special charaters not supported by ROS with an '_'.
  static std::string rosifyName(const std::string & name);

  /// Returns true if the FMU of this instances supports a variable communication step-size.
  bool canHandleVariableCommunicationStepSize() const;

  /// Returns the default experiment step-size of the FMU of this instance.
  rclcpp::Duration getDefaultExperimentStep() const;

  /// Returns all variables (including parameters, aliases, etc.) of the wrapped FMU in the FMI
  /// Library's internal representation.
  std::vector<fmi2_import_variable_t *> getAllVariables() const;

  /// Returns all input variables of the wrapped FMU in the FMI Library's internal representation.
  std::vector<fmi2_import_variable_t *> getInputVariables() const;

  /// Returns all output variables of the wrapped FMU in the FMI Library's internal representation.
  std::vector<fmi2_import_variable_t *> getOutputVariables() const;

  /// Returns all parameters of the wrapped FMU in the FMI Library's internal representation.
  std::vector<fmi2_import_variable_t *> getParameters() const;

  /// Returns the names of all variables (including parameters and aliases) of the wrapped FMU in
  /// the FMI Library's internal representation.
  std::vector<std::string> getAllVariableNames() const;

  /// Returns the names of all input variables of the wrapped FMU in the FMI Library's internal
  /// representation.
  std::vector<std::string> getInputVariableNames() const;

  /// Returns the names of all output variables of the wrapped FMU in the FMI Library's internal
  /// representation.
  std::vector<std::string> getOutputVariableNames() const;

  /// Returns the names of all parameters of the wrapped FMU in the FMI Library's internal
  /// representation.
  std::vector<std::string> getParameterNames() const;

  /// Stores a value for the given variable to be considered by doStep*(..) at the given time of
  /// the FMU simulation.
  void setInputValue(fmi2_import_variable_t * variable, const rclcpp::Time & time, double value);

  /// Stores a value for the variable with the given name to be considered by doStep*(..) at the
  /// given time of the FMU simulation.
  void setInputValue(const std::string & variableName, const rclcpp::Time & time, double value);

  /// Returns the step-size used in the FMU simulation.
  rclcpp::Duration getStepSize() const {return stepSize_;}

  /// Returns true if the wrapped FMU is still in initialization mode, which allows to set
  /// parameters and initial values.
  bool isInInitializationMode() const {return inInitializationMode_;}

  /// Exits the initialization mode and starts the simulation of the wrapped FMU. Uses the given
  /// timestamp as start time for the simulation whereas the FMU internally starts at time 0.
  /// All times passed to setValue(..) and doStep*(..) are translated correspondingly.
  void exitInitializationMode(const rclcpp::Time & simulationTime);

  /// Performs one simulation step using the configured step size and returns the current
  /// simulation time.
  rclcpp::Time doStep();

  /// Performs one simulation step using the given step size and returns the current simulation
  /// time.
  rclcpp::Time doStep(const rclcpp::Duration & stepSize);

  /// Advances the simulation of the wrapped FMU until the given point in time (modulo step-size).
  /// In detail, the simulation is performed iteratively using the configured step-size. Before
  /// each simulation step the relevant input values passed previously by setInputValue(..) are
  /// set depending on the given timestamps.
  rclcpp::Time doStepsUntil(const rclcpp::Time & simulationTime);

  /// Returns the current simulation time.
  rclcpp::Time getSimulationTime() const;

  /// Returns the current value of the given output variable.
  double getOutputValue(fmi2_import_variable_t * variable) const;

  /// Returns the current value of the output variable with the given name.
  double getOutputValue(const std::string & variableName) const;

  /// Sets the given value of the given variable (or parameter or alias) as initial values. This
  /// function may be called only while isInInitializationMode() = true.
  void setInitialValue(fmi2_import_variable_t * variable, double value);

  /// Sets the given value of the variable (or parameter or alias) with the given name as initial
  /// values. This function may be called only while isInInitializationMode() = true.
  void setInitialValue(const std::string & variableName, double value);

  /// Declares a ROS parameter for each FMU variable, parameter and alias. Note that ROS
  /// parameter names may use the characters [A-Za-z0-9_] only. Therefore, all other characters
  /// in an FMU variable name are mapped to '_'. For example, the FMU variable 'dx[2]' is named
  /// '/my_node_name/dx_2_' as a ROS parameter.
  void declareROSParameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nodeInterface);

  /// Tries to read inital values for each variable (including parameters and aliases) from the
  /// ROS parameter set. This function requires that ROS parameters have been declared for the
  /// FMU variables, parameters and aliases first using declareROSParameters(..).
  void initializeFromROSParameters(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nodeInterface);

private:
  /// The logger to be used for debug and error message.
  rclcpp::Logger logger_;

  /// Path of the FMU being wrapped by this instance.
  const std::string fmuPath_;

  /// Step size for the FMU simulation.
  rclcpp::Duration stepSize_;

  /// States whether between input values should be considered as continuous, piecewise linear
  /// signal (true) or as non-continuous, piecewise constant signal (false).
  bool interpolateInput_;

  /// Path to folder for extracting the FMU file temporarilly.
  std::string tmpPath_;

  /// In case that a random folder /tmp is being used (i.e. if given tmp path is ""), clean up
  /// this folder in dtor.
  bool removeTmpPathInDtor_{false};

  bool inInitializationMode_{true};

  /// Offset between the FMU's simulation time and the ROS-based simulation time for doStep*(..)
  /// and setValue(..)
  rclcpp::Duration fmuTimeOffset_{0};

  /// The current FMU's simulation time. It is fmuTime_ = simulationTime_ - fmuTimeOffset_.
  double fmuTime_{0.0};

  /// The current ROS-based simulation time. It is fmuTime_ = simulationTime_ - fmuTimeOffset_.
  rclcpp::Time simulationTime_{0, 0, RCL_ROS_TIME};

  /// Pointer from the FMU Library types to the FMU instance.
  fmi2_import_t * fmu_{nullptr};

  /// Pointer from the FMU Library types to the FMU context.
  fmi_import_context_t * context_{nullptr};

  /// Callback functions for FMI Library. Cannot use type fmi2_callback_functions_t here as it is
  /// a typedef of an anonymous struct,
  /// cf. https://stackoverflow.com/questions/7256436/forward-declarations-of-unnamed-struct
  void * fmiCallbacks_{nullptr};

  /// Further callback functions for FMI Library.
  jm_callbacks * jmCallbacks_{nullptr};

  /// Stores the mapping from timestamps to variable values for the FMU simulation.
  std::map<fmi2_import_variable_t *, std::map<rclcpp::Time, double>> inputValuesByVariable_{};

  /// Performs one simulation step using the given step size. Argument and state w.r.t.
  /// initialization mode are not checked.
  void doStepInternal(const rclcpp::Duration & stepSize);

  /// Returns the current simulation time. The state w.r.t. initialization mode is not checked.
  rclcpp::Time getSimulationTimeInternal() const
  {
    return rclcpp::Time(static_cast<uint64_t>(fmuTime_ * 1000000000.0),
             RCL_ROS_TIME) + fmuTimeOffset_;
  }
};

}  // namespace fmi_adapter

#endif  // FMI_ADAPTER__FMIADAPTER_HPP_
