// Copyright (c) 2018 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschresearch/fmi_adapter.
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

#include <cassert>

#include <exception>
#include <map>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Float64.h>

#include "fmi_adapter/FMIAdapter.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "fmi_adapter_node");
  ros::NodeHandle n("~");

  std::string fmuPath;
  if (!n.getParam("fmu_path", fmuPath)) {
    ROS_ERROR("Parameter 'fmu_path' not specified!");
    throw std::runtime_error("Parameter 'fmu_path' not specified!");
  }

  double stepSizeAsDouble = 0.0;
  n.getParam("step_size", stepSizeAsDouble);
  ros::Duration stepSize(stepSizeAsDouble);

  fmi_adapter::FMIAdapter adapter(fmuPath, stepSize);
  for (const std::string name : adapter.getParameterNames()) {
    ROS_DEBUG("FMU has parameter '%s'", name.c_str());
  }
  adapter.initializeFromROSParameters(n);

  std::map<std::string, ros::Subscriber> subscribers;
  for (const std::string& name : adapter.getInputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    ros::Subscriber subscriber =
        n.subscribe<std_msgs::Float64>(rosifiedName, 1000, [&adapter, name](const std_msgs::Float64::ConstPtr& msg) {
          std::string myName = name;
          adapter.setInputValue(myName, ros::Time::now(), msg->data);
        });
    subscribers[name] = subscriber;
  }

  std::map<std::string, ros::Publisher> publishers;
  for (const std::string& name : adapter.getOutputVariableNames()) {
    std::string rosifiedName = fmi_adapter::FMIAdapter::rosifyName(name);
    publishers[name] = n.advertise<std_msgs::Float64>(rosifiedName, 1000);
  }

  adapter.exitInitializationMode(ros::Time::now());

  ros::Timer timer = n.createTimer(ros::Duration(0.01), [&](const ros::TimerEvent& event) {
    if (adapter.getSimulationTime() < event.current_expected) {
      adapter.calcUntil(event.current_expected);
    } else {
      ROS_INFO("Simulation time %f is greater than timer's time %f. Is your step size to large?",
               adapter.getSimulationTime().toSec(), event.current_expected.toSec());
    }
    for (const std::string& name : adapter.getOutputVariableNames()) {
      std_msgs::Float64 msg;
      msg.data = adapter.getOutputValue(name);
      publishers[name].publish(msg);
    }
  });

  ros::spin();

  return 0;
}
