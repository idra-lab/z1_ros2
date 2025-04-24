cmake_minimum_required(VERSION 3.8)
project(z1_examples)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake        REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy              REQUIRED)

install(
    PROGRAMS
    ${PROJECT_NAME}/waypoint_test.py
    DESTINATION 
    lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch 
    DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
ament_package()
