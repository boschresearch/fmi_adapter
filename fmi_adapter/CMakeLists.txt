cmake_minimum_required(VERSION 3.5)

project(fmi_adapter)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

include(ExternalProject)
externalproject_add(FMILibraryProject
  URL https://jmodelica.org/fmil/FMILibrary-2.0.3-src.zip
  URL_MD5 53d8edd7442c31bcd3fb56477e574845
  TIMEOUT 60
)
externalproject_get_property(FMILibraryProject INSTALL_DIR)
set(FMILibraryProject_INCLUDE_DIR "${INSTALL_DIR}/src/install/include")
set(FMILibraryProject_LIB_DIR "${INSTALL_DIR}/src/install/lib")

add_library(${PROJECT_NAME}_fmilib STATIC IMPORTED)
set_target_properties(${PROJECT_NAME}_fmilib PROPERTIES IMPORTED_LOCATION ${FMILibraryProject_LIB_DIR}/libfmilib.a)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/${PROJECT_NAME}/FMIAdapter.cpp
  src/${PROJECT_NAME}/FMIAdapterNode.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
)
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_fmilib dl)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})

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
    "std_msgs"
    "rcutils"
  )

  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # No copyright header check since link to NOTICE file is not recognized properly.
  set(ament_cmake_cppcheck_FOUND TRUE)  # TODO(Ralph) Fix failing of cpplint at TEST macro of Googletest.
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp)
ament_export_dependencies(std_msgs)
ament_package()
