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

#include "fmi_adapter/FMIAdapterNode.hpp"

#include <cassert>

#include <chrono>
#include <map>
#include <memory>
#include <string>

#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "fmi_adapter/FMIAdapter.hpp"

namespace fmi_adapter
{

FMIAdapterNode::FMIAdapterNode(const rclcpp::NodeOptions & options)
: LifecycleNode("fmi_adapter_node", options)
{
  get_node_parameters_interface()->declare_parameter("fmu_path", rclcpp::ParameterValue(
      ""), rcl_interfaces::msg::ParameterDescriptor());
  get_node_parameters_interface()->declare_parameter("step_size", rclcpp::ParameterValue(
      0.0), rcl_interfaces::msg::ParameterDescriptor());
  get_node_parameters_interface()->declare_parameter("update_period", rclcpp::ParameterValue(
      0.01), rcl_interfaces::msg::ParameterDescriptor());
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FMIAdapterNode::on_configure(const rclcpp_lifecycle::State &)
{
  std::string fmuPath;
  rclcpp::Parameter parameter;
  if (get_parameter("fmu_path", parameter)) {
    fmuPath = parameter.as_string();
  } else {
    RCLCPP_ERROR(get_logger(), "Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  double stepSizeAsDouble = 0.0;
  if (get_parameter("step_size", parameter)) {
    stepSizeAsDouble = parameter.as_double();
  }
  rclcpp::Duration stepSize = rclcpp::Duration(1, 0) * stepSizeAsDouble;

  adapter_ = std::make_shared<fmi_adapter::FMIAdapter>(get_logger(), fmuPath, stepSize);
  for (const std::string & name : adapter_->getParameterNames()) {
    RCLCPP_DEBUG(get_logger(), "FMU has parameter '%s'", name.c_str());
  }
  adapter_->declareROSParameters(get_node_parameters_interface());
  adapter_->initializeFromROSParameters(get_node_parameters_interface());

  for (const std::string & name : adapter_->getInputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    auto subscription =
      create_subscription<std_msgs::msg::Float64>(rosifiedName, rclcpp::SystemDefaultsQoS(),
        [this, name](const std_msgs::msg::Float64::SharedPtr msg) {
          std::string myName = name;
          adapter_->setInputValue(myName, now(), msg->data);
        });
    subscriptions_[name] = subscription;
  }

  for (const std::string & name : adapter_->getOutputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    publishers_[name] = create_publisher<std_msgs::msg::Float64>(rosifiedName,
        rclcpp::SystemDefaultsQoS());
  }

  adapter_->exitInitializationMode(now());

  std::chrono::nanoseconds updatePeriod(10000000);  // Default is 0.01s
  if (get_parameter("update_period", parameter)) {
    updatePeriod =
      std::chrono::nanoseconds(static_cast<int64_t>(parameter.as_double() * 1000000000.0));
  }

  timer_ = create_wall_timer(updatePeriod, [this]() {
        rclcpp::Time currentTimepoint = now();
        if (adapter_->getSimulationTime() < currentTimepoint) {
          adapter_->doStepsUntil(currentTimepoint);
        } else {
          RCLCPP_INFO(get_logger(),
          "Simulation time %f is greater than timer's time %f. Is your step size to large?",
          adapter_->getSimulationTime().seconds(), currentTimepoint.seconds());
        }
        for (const std::string & name : adapter_->getOutputVariableNames()) {
          std_msgs::msg::Float64 msg;
          msg.data = adapter_->getOutputValue(name);
          if (publishers_[name]->is_activated()) {
            publishers_[name]->publish(msg);
          }
        }
      });

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FMIAdapterNode::on_activate(const rclcpp_lifecycle::State &)
{
  for (auto pair : publishers_) {
    pair.second->on_activate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FMIAdapterNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto pair : publishers_) {
    pair.second->on_deactivate();
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FMIAdapterNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  subscriptions_.clear();
  publishers_.clear();

  adapter_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
FMIAdapterNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  timer_.reset();
  subscriptions_.clear();
  publishers_.clear();

  adapter_.reset();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace fmi_adapter

RCLCPP_COMPONENTS_REGISTER_NODE(fmi_adapter::FMIAdapterNode)
