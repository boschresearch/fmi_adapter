cmake_minimum_required(VERSION 3.5)

project(fmi_adapter_examples)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY share DESTINATION share/${PROJECT_NAME}/)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(fmi_adapter)
ament_package()

