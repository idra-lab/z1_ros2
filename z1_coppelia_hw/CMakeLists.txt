cmake_minimum_required(VERSION 3.16)
project(z1_coppelia_hw LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  z1_coppelia_hw
  SHARED
  hardware/HWInterface.cpp
)
target_compile_features(z1_coppelia_hw PUBLIC cxx_std_17)
target_include_directories(z1_coppelia_hw PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/z1_coppelia_hw>
)
ament_target_dependencies(
  z1_coppelia_hw PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "ROS2_CONTROL_DEMO_EXAMPLE_1_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface z1_coppelia_hw.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/z1_coppelia_hw
)

install(
  DIRECTORY bringup/launch config 
  DESTINATION share/z1_coppelia_hw
)

install(TARGETS z1_coppelia_hw
  EXPORT export_franka_coppelia_hw
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_franka_coppelia_hw HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
