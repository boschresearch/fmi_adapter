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
#include <iostream>
#include <gtest/gtest.h>
#include <ros/ros.h>


int main(int argc, char** argv) {
  ros::init(argc, argv, "fmi_adapter_test", ros::init_options::AnonymousName);

  if (!ros::master::check()) {
    std::cout << "ROS master is NOT running. Skipping all unit tests!" << std::endl;
    ::testing::GTEST_FLAG(filter) = "-*";
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  }

  ros::start();

  testing::InitGoogleTest(&argc, argv);
  int testResult = RUN_ALL_TESTS();

  return testResult;
}
