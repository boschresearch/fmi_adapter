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

#ifndef FMI_ADAPTER__FMIADAPTERNODE_HPP_
#define FMI_ADAPTER__FMIADAPTERNODE_HPP_

#include <cassert>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>

namespace fmi_adapter
{

class FMIAdapter;

class FMIAdapterNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit FMIAdapterNode(const rclcpp::NodeOptions & options);

  RCLCPP_DISABLE_COPY(FMIAdapterNode)

  virtual ~FMIAdapterNode() = default;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &);

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &);

private:
  std::shared_ptr<fmi_adapter::FMIAdapter> adapter_{};

  rclcpp::TimerBase::SharedPtr timer_{};

  std::map<std::string,
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Float64>>> subscriptions_{};

  std::map<std::string,
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64>>> publishers_;
};

}  // namespace fmi_adapter

#endif  // FMI_ADAPTER__FMIADAPTERNODE_HPP_
