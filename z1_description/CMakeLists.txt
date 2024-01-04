cmake_minimum_required(VERSION 3.8)
project(z1_description)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(ROS2_REQUIRED_COMPONENTS
    ament_cmake
    )

foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()

install(
    DIRECTORY launch config urdf meshes rviz
    DESTINATION share/${PROJECT_NAME}
    )

ament_package()
