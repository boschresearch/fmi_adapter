cmake_minimum_required(VERSION 3.5)

project(fmi_adapter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(fmilibrary_vendor REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/FMIAdapter.cpp
  src/${PROJECT_NAME}/FMIAdapterNode.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include> PRIVATE $<BUILD_INTERFACE:${FMILibraryProject_INCLUDE_DIR}>)
ament_target_dependencies(${PROJECT_NAME}
  "rcl_interfaces"
  "rclcpp"
  "rclcpp_components"
  "rclcpp_lifecycle"
  "std_msgs"
  "fmilibrary_vendor"
)
target_link_libraries(${PROJECT_NAME} dl)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
rclcpp_components_register_nodes(${PROJECT_NAME} "fmi_adapter::FMIAdapterNode")

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test
    test/${PROJECT_NAME}/FMIAdapterTest.cpp
    ENV test_fmus_path=${CMAKE_CURRENT_SOURCE_DIR}/test/fmu/
  )
  target_include_directories(${PROJECT_NAME}_test PRIVATE include)
  target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
  ament_target_dependencies(${PROJECT_NAME}_test
    "rclcpp"
    "rclcpp_lifecycle"
    "rcutils"
    "std_msgs"
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # No copyright header check since link to NOTICE file is not recognized properly.
  set(ament_cmake_cppcheck_FOUND TRUE)  # TODO(Ralph) Fix failing of cpplint at TEST macro of Googletest.
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(rclcpp_lifecycle)
ament_export_dependencies(std_msgs)
ament_package()
