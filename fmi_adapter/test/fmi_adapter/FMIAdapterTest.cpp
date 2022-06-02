// Copyright (c) 2019 - for information on the respective copyright owner
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

#include <chrono>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include <rcutils/env.h>

#include "fmi_adapter/FMIAdapter.hpp"


namespace
{

const double EPSILON = 0.01;

using namespace std::chrono_literals;


class FMIAdapterTest : public ::testing::Test, public rclcpp::Node
{
public:
  FMIAdapterTest()
  : rclcpp::Node("fmi_adapter_test_node")
  {
    const char * testFMUsPath;
    const char * errorMsg = rcutils_get_env("test_fmus_path", &testFMUsPath);
    if (errorMsg) {
      throw std::runtime_error(std::string("Env variable test_fmus_path not found: ") + errorMsg);
    }
    test_FMUs_path_ = testFMUsPath;
  }

  std::string test_FMUs_path_ = "";

  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};


// The FMU TransportDelay.fmu is built for x86-64 (AMD64). Therefore, run unit
// tests on this architecture only.
#ifdef __x86_64__


TEST_F(FMIAdapterTest, ctorWithImplicitDefaultExperimentStepSize) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_EQ(wrapper.getStepSize(), rclcpp::Duration(1ms));
}


TEST_F(FMIAdapterTest, ctorWithExplicitDefaultExperimentStepSize) {
  fmi_adapter::FMIAdapter wrapper(
    get_logger(), test_FMUs_path_ + "TransportDelay.fmu", rclcpp::Duration(0ms));
  EXPECT_EQ(wrapper.getStepSize(), rclcpp::Duration(1ms));
}


TEST_F(FMIAdapterTest, ctorWithExplicitStepSize) {
  fmi_adapter::FMIAdapter wrapper(
    get_logger(), test_FMUs_path_ + "TransportDelay.fmu", rclcpp::Duration(4ms));
  EXPECT_EQ(wrapper.getStepSize(), rclcpp::Duration(4ms));
}


TEST_F(FMIAdapterTest, canHandleVariableCommunicationStepSize) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_FALSE(wrapper.canHandleVariableCommunicationStepSize());
}


TEST_F(FMIAdapterTest, getDefaultExperimentStep) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_EQ(wrapper.getDefaultExperimentStep(), rclcpp::Duration(1ms));
}


TEST_F(FMIAdapterTest, getAllVariableNames) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  wrapper.exitInitializationMode(rclcpp::Time(17, 0, RCL_ROS_TIME));
  std::vector<std::string> expectedNames = {"x", "y", "d"};
  std::vector<std::string> actualNames = wrapper.getAllVariableNames();
  EXPECT_EQ(expectedNames, actualNames);
}


TEST_F(FMIAdapterTest, doStepsUntil_withInterpolation) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  const rclcpp::Duration DELAY(2, 0);

  wrapper.setInitialValue("x", 2.0);
  const rclcpp::Time startTime = rclcpp::Time(17, 0, RCL_ROS_TIME);
  wrapper.exitInitializationMode(startTime);

  wrapper.setInputValue("x", startTime + rclcpp::Duration(4000ms), 2.0);
  wrapper.setInputValue("x", startTime + rclcpp::Duration(7000ms), 3.0);
  wrapper.setInputValue("x", startTime + rclcpp::Duration(9000ms), -1.0);

  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(500ms));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(4500ms));
  EXPECT_NEAR(2.167, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(7000ms));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(8500ms));
  EXPECT_NEAR(0.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(9500ms));
  EXPECT_NEAR(-1.0, wrapper.getOutputValue("y"), EPSILON);
}


TEST_F(FMIAdapterTest, doStepsUntil_withoutInterpolation) {
  fmi_adapter::FMIAdapter wrapper(
    get_logger(), test_FMUs_path_ + "TransportDelay.fmu", rclcpp::Duration(1ms), false);
  const rclcpp::Duration DELAY(2000ms);

  wrapper.setInitialValue("x", 2.0);
  const rclcpp::Time startTime = rclcpp::Time(17, 0, RCL_ROS_TIME);
  wrapper.exitInitializationMode(startTime);

  wrapper.setInputValue("x", startTime + rclcpp::Duration(4000ms), 2.0);
  wrapper.setInputValue("x", startTime + rclcpp::Duration(7000ms), 3.0);
  wrapper.setInputValue("x", startTime + rclcpp::Duration(9000ms), -1.0);

  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(500ms));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(4500ms));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(7100ms));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(8500ms));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(9500ms));
  EXPECT_NEAR(-1.0, wrapper.getOutputValue("y"), EPSILON);
}


TEST_F(FMIAdapterTest, doStepsUntil_interpolationAfterExtrapolation) {
  fmi_adapter::FMIAdapter wrapper(get_logger(), test_FMUs_path_ + "TransportDelay.fmu");
  const rclcpp::Duration DELAY(2, 0);

  wrapper.setInitialValue("x", 2.0);
  const rclcpp::Time startTime = rclcpp::Time(17, 0, RCL_ROS_TIME);
  wrapper.exitInitializationMode(startTime);

  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(1000ms));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.setInputValue("x", startTime + rclcpp::Duration(4000ms), 1.0);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(2900ms));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + rclcpp::Duration(3500ms));
  EXPECT_NEAR(1.125, wrapper.getOutputValue("y"), EPSILON);
}


#endif  // __x86_64__


}  // namespace
