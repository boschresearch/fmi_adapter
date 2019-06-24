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

#include "fmi_adapter/FMIAdapter.hpp"

#include <cassert>
#include <cmath>
#include <cstdlib>

#include <algorithm>
#include <exception>
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <dirent.h>
#include <unistd.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <fmilib.h>
#pragma GCC diagnostic pop

namespace fmi_adapter
{

namespace helpers
{

bool canWriteToFolder(const std::string & path)
{
  DIR * dir = opendir(path.c_str());
  if (dir == nullptr) {
    return false;
  }
  closedir(dir);
  return access(path.c_str(), W_OK) == 0;
}


bool canReadFromFile(const std::string & path)
{
  std::ifstream stream(path.c_str());
  if (stream.is_open() && stream.good()) {
    stream.close();
    return true;
  } else {
    return false;
  }
}


bool variableFilterAll(__attribute__((unused)) fmi2_import_variable_t * variable) {return true;}


bool variableFilterByCausality(fmi2_import_variable_t * variable, fmi2_causality_enu_t causality)
{
  return fmi2_import_get_causality(variable) == causality;
}


/// This helper functions returns all variables with a certain property defined by a filter.
std::vector<fmi2_import_variable_t *> getVariablesFromFMU(
  fmi2_import_t * fmu, std::function<bool(fmi2_import_variable_t *)> filter = variableFilterAll)
{
  assert(fmu);

  std::vector<fmi2_import_variable_t *> result;

  fmi2_import_variable_list_t * variableList = fmi2_import_get_variable_list(fmu, 0);
  const size_t variablesCount = fmi2_import_get_variable_list_size(variableList);
  for (size_t index = 0; index < variablesCount; ++index) {
    fmi2_import_variable_t * variable = fmi2_import_get_variable(variableList, index);
    if (filter(variable)) {
      result.push_back(variable);
    }
  }

  fmi2_import_free_variable_list(variableList);

  return result;
}


/// This helper functions returns all variable names with a certain property defined by a filter.
std::vector<std::string> getVariableNamesFromFMU(
  fmi2_import_t * fmu, std::function<bool(fmi2_import_variable_t *)> filter = variableFilterAll)
{
  assert(fmu);

  std::vector<std::string> result;

  fmi2_import_variable_list_t * variableList = fmi2_import_get_variable_list(fmu, 0);
  const size_t variablesCount = fmi2_import_get_variable_list_size(variableList);
  for (size_t index = 0; index < variablesCount; ++index) {
    fmi2_import_variable_t * variable = fmi2_import_get_variable(variableList, index);
    if (filter(variable)) {
      std::string name = fmi2_import_get_variable_name(variable);
      result.push_back(name);
    }
  }

  fmi2_import_free_variable_list(variableList);

  return result;
}

}  // namespace helpers


FMIAdapter::FMIAdapter(
  rclcpp::Logger logger, const std::string & fmuPath, rclcpp::Duration stepSize,
  bool interpolateInput,
  const std::string & tmpPath)
: logger_(logger), fmuPath_(fmuPath), stepSize_(stepSize), interpolateInput_(interpolateInput),
  tmpPath_(tmpPath)
{
  if (stepSize == rclcpp::Duration(0)) {
    // Use step-size from FMU. See end of ctor.
  } else if (stepSize < rclcpp::Duration(0)) {
    throw std::invalid_argument("Step size must be positive!");
  }
  if (!helpers::canReadFromFile(fmuPath)) {
    throw std::invalid_argument("Given FMU file '" + fmuPath + "' not found or cannot be read!");
  }
  if (tmpPath_.empty()) {
    char pathPattern[] = "/tmp/fmi_adapter_XXXXXX";
    tmpPath_ = mkdtemp(pathPattern);
    removeTmpPathInDtor_ = true;
  }
  if (!helpers::canWriteToFolder(tmpPath_)) {
    throw std::invalid_argument("Cannot access tmp folder '" + tmpPath_ + "'!");
  }

  // Some of the following lines have been taken from FMILibrary 2.3.0
  // src/test/fmi2_import_cs_test.c under BSD style license.

  jmCallbacks_ = new jm_callbacks;
  jmCallbacks_->malloc = malloc;
  jmCallbacks_->calloc = calloc;
  jmCallbacks_->realloc = realloc;
  jmCallbacks_->free = free;
  jmCallbacks_->logger = jm_default_logger;
  jmCallbacks_->log_level = jm_log_level_error;
  jmCallbacks_->context = 0;

  context_ = fmi_import_allocate_context(jmCallbacks_);

  fmi_version_enu_t fmuVersion = fmi_import_get_fmi_version(context_,
      fmuPath_.c_str(), tmpPath_.c_str());
  if (fmuVersion != fmi_version_2_0_enu) {
    throw std::invalid_argument(
            "Could not load the FMU or the FMU does not meet the FMI 2.0 standard!");
  }

  fmu_ = fmi2_import_parse_xml(context_, tmpPath_.c_str(), 0);
  if (!fmu_) {
    throw std::invalid_argument("Could not parse XML description of FMU!");
  }

  if (fmi2_import_get_fmu_kind(fmu_) != fmi2_fmu_kind_cs) {
    throw std::invalid_argument("Given FMU is not for co-simulation!");
  }

  fmi2_callback_functions_t * fmiCallbacks = new fmi2_callback_functions_t;
  fmiCallbacks->logger = fmi2_log_forwarding;
  fmiCallbacks->allocateMemory = calloc;
  fmiCallbacks->freeMemory = free;
  fmiCallbacks->componentEnvironment = fmu_;
  fmiCallbacks_ = fmiCallbacks;

  jm_status_enu_t jmStatus = fmi2_import_create_dllfmu(fmu_, fmi2_fmu_kind_cs, fmiCallbacks);
  if (jmStatus == jm_status_error) {
    throw std::runtime_error("Creation of dllfmu failed!");
  }

  const fmi2_string_t instanceName = fmi2_import_get_model_name(fmu_);
  const fmi2_string_t fmuLocation = "";
  const fmi2_boolean_t visible = fmi2_false;
  const fmi2_real_t relativeTol = 1e-4;
  jmStatus = fmi2_import_instantiate(fmu_, instanceName, fmi2_cosimulation, fmuLocation, visible);
  assert(jmStatus != jm_status_error);

  const fmi2_real_t startTime = 0.0;
  const fmi2_real_t stopTime = -1.0;
  fmi2_status_t fmiStatus = fmi2_import_setup_experiment(fmu_, fmi2_true, relativeTol, startTime,
      fmi2_false, stopTime);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_setup_experiment failed!");
  }

  fmiStatus = fmi2_import_enter_initialization_mode(fmu_);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_enter_initialization_mode failed!");
  }

  if (stepSize == rclcpp::Duration(0)) {
    stepSize_ = rclcpp::Duration(1, 0) * fmi2_import_get_default_experiment_step(fmu_);
    if (stepSize_ <= rclcpp::Duration(0)) {
      throw std::invalid_argument("Default experiment step size from FMU is not positive!");
    }
    RCLCPP_INFO(logger_, "No step-size argument given. Using default from FMU, which is %fs.",
      stepSize_.seconds());
  }
}


FMIAdapter::~FMIAdapter()
{
  if (!inInitializationMode_) {
    fmi2_import_terminate(fmu_);
  }
  fmi2_import_free_instance(fmu_);
  fmi2_import_destroy_dllfmu(fmu_);
  fmi2_import_free(fmu_);
  fmi_import_free_context(context_);
  delete jmCallbacks_;
  delete static_cast<fmi2_callback_functions_t *>(fmiCallbacks_);

  if (removeTmpPathInDtor_) {
    // TODO(Ralph) Remove folder fmi_adapter_XXXXXX from /tmp.
    // Such function is not provided by Posix or C++11/14.
    // Possibly use boost::filesystem::remove_all. Then other
    // filesystem functions used here and use of "/tmp" may be
    // replaced by corresponding functions from boost::filesystem.
  }
}


std::string FMIAdapter::rosifyName(const std::string & name)
{
  std::string result = name;
  for (size_t i = 0; i < result.size(); ++i) {
    char c = result[i];
    if (('a' <= c && c <= 'z') || ('A' <= c && c <= 'Z') || ('0' <= c && c <= '9') || c == '_') {
      // Keep valid char
    } else {
      result[i] = '_';
    }
  }

  while (result.length() > 0 && result[0] == '_') {
    result.erase(0, 1);
  }

  return result;
}


bool FMIAdapter::canHandleVariableCommunicationStepSize() const
{
  return static_cast<bool>(fmi2_import_get_capability(fmu_,
         fmi2_cs_canHandleVariableCommunicationStepSize));
}


rclcpp::Duration FMIAdapter::getDefaultExperimentStep() const
{
  return rclcpp::Duration(1, 0) * fmi2_import_get_default_experiment_step(fmu_);
}


std::vector<fmi2_import_variable_t *> FMIAdapter::getAllVariables() const
{
  return helpers::getVariablesFromFMU(fmu_, helpers::variableFilterAll);
}


std::vector<fmi2_import_variable_t *> FMIAdapter::getInputVariables() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_input);
  return helpers::getVariablesFromFMU(fmu_, filter);
}


std::vector<fmi2_import_variable_t *> FMIAdapter::getOutputVariables() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_output);
  return helpers::getVariablesFromFMU(fmu_, filter);
}


std::vector<fmi2_import_variable_t *> FMIAdapter::getParameters() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_parameter);
  return helpers::getVariablesFromFMU(fmu_, filter);
}


std::vector<std::string> FMIAdapter::getAllVariableNames() const
{
  return helpers::getVariableNamesFromFMU(fmu_, helpers::variableFilterAll);
}


std::vector<std::string> FMIAdapter::getInputVariableNames() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_input);
  return helpers::getVariableNamesFromFMU(fmu_, filter);
}


std::vector<std::string> FMIAdapter::getOutputVariableNames() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_output);
  return helpers::getVariableNamesFromFMU(fmu_, filter);
}


std::vector<std::string> FMIAdapter::getParameterNames() const
{
  auto filter = std::bind(helpers::variableFilterByCausality, std::placeholders::_1,
      fmi2_causality_enu_parameter);
  return helpers::getVariableNamesFromFMU(fmu_, filter);
}


void FMIAdapter::exitInitializationMode(const rclcpp::Time & simulationTime)
{
  if (!inInitializationMode_) {
    throw std::runtime_error("FMU is no longer in initialization mode!");
  }

  fmi2_status_t fmiStatus = fmi2_import_exit_initialization_mode(fmu_);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_exit_initialization_mode failed!");
  }
  inInitializationMode_ = false;

  fmuTimeOffset_ = simulationTime - rclcpp::Time(0, 0, RCL_ROS_TIME);
  assert(fmuTime_ == 0.0);

  // TODO(Ralph) Avoid creation of std::vector here.
  for (fmi2_import_variable_t * variable : getInputVariables()) {
    std::map<rclcpp::Time, double> & inputValues = inputValuesByVariable_[variable];
    if (inputValues.empty() || inputValues.begin()->first > simulationTime) {
      fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
      fmi2_real_t value;
      fmi2_import_get_real(fmu_, &valueReference, 1, &value);
      inputValues[simulationTime] = value;
    }
  }
}


rclcpp::Time FMIAdapter::doStep()
{
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  doStepInternal(stepSize_);

  return getSimulationTimeInternal();
}


rclcpp::Time FMIAdapter::doStep(const rclcpp::Duration & stepSize)
{
  if (stepSize <= rclcpp::Duration(0)) {
    throw std::invalid_argument("Step size must be positive!");
  }
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  doStepInternal(stepSize);

  return getSimulationTimeInternal();
}


void FMIAdapter::doStepInternal(const rclcpp::Duration & stepSize)
{
  // TODO(Ralph) Avoid creation of std::vector here.
  for (fmi2_import_variable_t * variable : getInputVariables()) {
    std::map<rclcpp::Time, double> & inputValues = inputValuesByVariable_[variable];
    assert(
      !inputValues.empty() &&
      (inputValues.begin()->first - fmuTimeOffset_).seconds() <= fmuTime_);
    while (inputValues.size() >= 2 &&
      (std::next(inputValues.begin())->first - fmuTimeOffset_).seconds() <= fmuTime_)
    {
      inputValues.erase(inputValues.begin());
    }
    assert(
      !inputValues.empty() &&
      (inputValues.begin()->first - fmuTimeOffset_).seconds() <= fmuTime_);
    double value = inputValues.begin()->second;
    if (interpolateInput_ && inputValues.size() > 1) {
      double t0 = (inputValues.begin()->first - fmuTimeOffset_).seconds();
      double t1 = (std::next(inputValues.begin())->first - fmuTimeOffset_).seconds();
      double weight = (t1 - fmuTime_) / (t1 - t0);
      double x0 = value;
      double x1 = std::next(inputValues.begin())->second;
      value = weight * x0 + (1.0 - weight) * x1;
    }

    fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
    fmi2_import_set_real(fmu_, &valueReference, 1, &value);
  }

  const fmi2_boolean_t doStep = fmi2_true;
  fmi2_status_t fmiStatus = fmi2_import_do_step(fmu_, fmuTime_, stepSize.seconds(), doStep);
  if (fmiStatus != fmi2_status_ok) {
    throw std::runtime_error("fmi2_import_do_step failed!");
  }
  fmuTime_ += stepSize.seconds();
}


rclcpp::Time FMIAdapter::doStepsUntil(const rclcpp::Time & simulationTime)
{
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  fmi2_real_t targetFMUTime = (simulationTime - fmuTimeOffset_).seconds();
  if (targetFMUTime < fmuTime_) {
    RCLCPP_ERROR(logger_, "Given time %f is before current simulation time %f!", targetFMUTime,
      fmuTime_);
    throw std::invalid_argument("Given time is before current simulation time!");
  }

  while (fmuTime_ + stepSize_.seconds() / 2.0 < targetFMUTime) {
    doStepInternal(stepSize_);
  }

  return getSimulationTimeInternal();
}


rclcpp::Time FMIAdapter::getSimulationTime() const
{
  if (inInitializationMode_) {
    throw std::runtime_error("FMU is still in initialization mode!");
  }

  return getSimulationTimeInternal();
}


void FMIAdapter::setInputValue(
  fmi2_import_variable_t * variable, const rclcpp::Time & time,
  double value)
{
  if (fmi2_import_get_causality(variable) != fmi2_causality_enu_input) {
    throw std::invalid_argument("Given variable is not an input variable!");
  }

  inputValuesByVariable_[variable].insert(std::make_pair(time, value));
}


void FMIAdapter::setInputValue(
  const std::string & variableName, const rclcpp::Time & time,
  double value)
{
  fmi2_import_variable_t * variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  setInputValue(variable, time, value);
}


double FMIAdapter::getOutputValue(fmi2_import_variable_t * variable) const
{
  if (fmi2_import_get_causality(variable) != fmi2_causality_enu_output) {
    throw std::invalid_argument("Given variable is not an output variable!");
  }

  fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
  fmi2_real_t value;
  fmi2_import_get_real(fmu_, &valueReference, 1, &value);

  return value;
}


double FMIAdapter::getOutputValue(const std::string & variableName) const
{
  fmi2_import_variable_t * variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  return getOutputValue(variable);
}


void FMIAdapter::setInitialValue(fmi2_import_variable_t * variable, double value)
{
  if (!inInitializationMode_) {
    throw std::runtime_error("Initial values can be only set in initialization mode!");
  }

  fmi2_value_reference_t valueReference = fmi2_import_get_variable_vr(variable);
  fmi2_import_set_real(fmu_, &valueReference, 1, &value);

  std::string name = fmi2_import_get_variable_name(variable);
  RCLCPP_INFO(logger_, "Set initial value of variable '%s' to %f", name.c_str(), value);
}


void FMIAdapter::setInitialValue(const std::string & variableName, double value)
{
  fmi2_import_variable_t * variable = fmi2_import_get_variable_by_name(fmu_, variableName.c_str());
  if (variable == nullptr) {
    throw std::invalid_argument("Unknown variable name!");
  }

  setInitialValue(variable, value);
}


void FMIAdapter::declareROSParameters(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nodeInterface)
{
  if (nodeInterface == nullptr) {
    throw std::invalid_argument("Pointer to parameter inferface must not be null!");
  }

  for (fmi2_import_variable_t * variable : helpers::getVariablesFromFMU(fmu_)) {
    std::string name = fmi2_import_get_variable_name(variable);
    name = rosifyName(name);
    nodeInterface->declare_parameter(name, rclcpp::ParameterValue(),
      rcl_interfaces::msg::ParameterDescriptor());
  }
}

void FMIAdapter::initializeFromROSParameters(
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr nodeInterface)
{
  if (nodeInterface == nullptr) {
    throw std::invalid_argument("Pointer to parameter inferface must not be null!");
  }

  for (fmi2_import_variable_t * variable : helpers::getVariablesFromFMU(fmu_)) {
    std::string name = fmi2_import_get_variable_name(variable);
    name = rosifyName(name);
    double value = 0.0;
    rclcpp::Parameter parameter;
    if (nodeInterface->get_parameter(name, parameter)) {
      if (parameter.get_type() == rclcpp::PARAMETER_NOT_SET) {
        // Use default value or initial guess from FMU.
      } else {
        value = parameter.as_double();
        setInitialValue(variable, value);
      }
    }
  }
}

}  // namespace fmi_adapter
