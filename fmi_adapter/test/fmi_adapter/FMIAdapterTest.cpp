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

#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <ros/ros.h>

#include "fmi_adapter/FMIAdapter.h"


namespace {

const double EPSILON = 0.01;


class FMIAdapterTest : public ::testing::Test {
 public:
  FMIAdapterTest() : handle_("~") {
    int trials = 0;
    while (!handle_.getParam("test_fmus_path", test_FMUs_path_)) {
      // roslaunch may require some time until parameter is set ...
      ++trials;
      if (trials > 10) {
        ROS_ERROR("Parameter 'test_fmus_path' not specified!");
        throw std::runtime_error("Parameter 'test_fmus_path' not specified!");
      }
      ros::Duration(0.1).sleep();
    }
  }

  ros::NodeHandle handle_;

  std::string test_FMUs_path_ = "";
};


// The FMU TransportDelay.fmu is built for x86-64 (AMD64). Therefore, run unit
// tests on this architecture only.
#ifdef __x86_64__


TEST_F(FMIAdapterTest, ctorWithImplicitDefaultExperimentStepSize) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_EQ(wrapper.getStepSize(), ros::Duration(0.001));
}


TEST_F(FMIAdapterTest, ctorWithExplicitDefaultExperimentStepSize) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu", ros::Duration(0.0));
  EXPECT_EQ(wrapper.getStepSize(), ros::Duration(0.001));
}


TEST_F(FMIAdapterTest, ctorWithExplicitStepSize) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu", ros::Duration(0.004));
  EXPECT_EQ(wrapper.getStepSize(), ros::Duration(0.004));
}


TEST_F(FMIAdapterTest, canHandleVariableCommunicationStepSize) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_FALSE(wrapper.canHandleVariableCommunicationStepSize());
}


TEST_F(FMIAdapterTest, getDefaultExperimentStep) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  EXPECT_EQ(wrapper.getDefaultExperimentStep(), ros::Duration(0.001));
}


TEST_F(FMIAdapterTest, getAllVariableNames) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  wrapper.exitInitializationMode(ros::Time::now());
  std::vector<std::string> expectedNames = {"x", "y", "d"};
  std::vector<std::string> actualNames = wrapper.getAllVariableNames();
  EXPECT_EQ(expectedNames, actualNames);
}


TEST_F(FMIAdapterTest, doStepsUntil_withInterpolation) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  const ros::Duration DELAY(2.0);

  wrapper.setInitialValue("x", 2.0);
  const ros::Time startTime = ros::Time::now();
  wrapper.exitInitializationMode(startTime);

  wrapper.setInputValue("x", startTime + ros::Duration(4.0), 2.0);
  wrapper.setInputValue("x", startTime + ros::Duration(7.0), 3.0);
  wrapper.setInputValue("x", startTime + ros::Duration(9.0), -1.0);

  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(0.5));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(4.5));
  EXPECT_NEAR(2.167, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(7.0));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(8.5));
  EXPECT_NEAR(0.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(9.5));
  EXPECT_NEAR(-1.0, wrapper.getOutputValue("y"), EPSILON);
}


TEST_F(FMIAdapterTest, doStepsUntil_withoutInterpolation) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu", ros::Duration(0.001), false);
  const ros::Duration DELAY(2.0);

  wrapper.setInitialValue("x", 2.0);
  const ros::Time startTime = ros::Time::now();
  wrapper.exitInitializationMode(startTime);

  wrapper.setInputValue("x", startTime + ros::Duration(4.0), 2.0);
  wrapper.setInputValue("x", startTime + ros::Duration(7.0), 3.0);
  wrapper.setInputValue("x", startTime + ros::Duration(9.0), -1.0);

  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(0.5));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(4.5));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(7.1));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(8.5));
  EXPECT_NEAR(3.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(9.5));
  EXPECT_NEAR(-1.0, wrapper.getOutputValue("y"), EPSILON);
}


TEST_F(FMIAdapterTest, doStepsUntil_interpolationAfterExtrapolation) {
  fmi_adapter::FMIAdapter wrapper(test_FMUs_path_ + "TransportDelay.fmu");
  const ros::Duration DELAY(2.0);

  wrapper.setInitialValue("x", 2.0);
  const ros::Time startTime = ros::Time::now();
  wrapper.exitInitializationMode(startTime);

  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(1.0));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.setInputValue("x", startTime + ros::Duration(4.0), 1.0);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(2.9));
  EXPECT_NEAR(2.0, wrapper.getOutputValue("y"), EPSILON);
  wrapper.doStepsUntil(startTime + DELAY + ros::Duration(3.5));
  EXPECT_NEAR(1.125, wrapper.getOutputValue("y"), EPSILON);
}


#endif  // __x86_64__


}  // namespace
