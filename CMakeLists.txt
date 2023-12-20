cmake_minimum_required(VERSION 3.8)
project(unitree_z1_hw_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
set(ROS2_REQUIRED_COMPONENTS
    ament_cmake
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    )

foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()


function(print_variable VAR)
    message(STATUS "${VAR} = ${${VAR}}")
endfunction()


#  _   _       _ _                   ____       _
# | | | |_ __ (_) |_ _ __ ___  ___  / ___|  ___| |_ _   _ _ __
# | | | | '_ \| | __| '__/ _ \/ _ \ \___ \ / _ \ __| | | | '_ \
# | |_| | | | | | |_| | |  __/  __/  ___) |  __/ |_| |_| | |_) |
#  \___/|_| |_|_|\__|_|  \___|\___| |____/ \___|\__|\__,_| .__/
#                                                        |_|

# Build the Z1 controller
add_subdirectory(unitree_z1_controller)

set(UNITREE_Z1_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/unitree_z1_sdk)
set(UNITREE_Z1_SDK_LIB_PATH ${UNITREE_Z1_SDK_PATH}/lib)
set(UNITREE_Z1_SDK_INCLUDE_PATH ${UNITREE_Z1_SDK_PATH}/include)
set(UNITREE_Z1_SDK_LIB ${UNITREE_Z1_SDK_LIB_PATH}/libZ1_SDK_${CMAKE_SYSTEM_PROCESSOR}.so)

set(Z1_CONTROLLER_FILE ${CMAKE_CURRENT_BINARY_DIR}/z1_ctrl)
configure_file(
    ${CMAKE_CURRENT_LIST_DIR}/include/unitree_z1_hw_interface/defines.hpp.in
    ${CMAKE_CURRENT_LIST_DIR}/include/unitree_z1_hw_interface/defines.hpp
    )
message(STATUS "z1_ctrl full path: ${Z1_CONTROLLER_FILE}")

#  _   ___        __  ___       _             __
# | | | \ \      / / |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___
# | |_| |\ \ /\ / /   | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
# |  _  | \ V  V /    | || | | | ||  __/ |  |  _| (_| | (_|  __/
# |_| |_|  \_/\_/    |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
#

set(UNITREE_Z1_HW_INTERFACE_SRCS
    src/unitree_z1_hw_interface.cpp
    # Add .cpp files here if needed
    )
add_library(
    ${PROJECT_NAME}
    SHARED
    ${UNITREE_Z1_HW_INTERFACE_SRCS}
    )
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
    PRIVATE
    ${UNITREE_Z1_SDK_INCLUDE_PATH}
    ${EIGEN3_INCLUDE_DIRS}
    )
print_variable(EIGEN3_INCLUDE_DIRS)
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${UNITREE_Z1_SDK_LIB}
    )
ament_target_dependencies(
    ${PROJECT_NAME}
    PUBLIC
    ${ROS2_REQUIRED_COMPONENTS}
    )
target_compile_options(
    ${PROJECT_NAME}
    PRIVATE
    -Wall
    -Wextra
    -Wpedantic
    -Wno-comment
    )

#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#

pluginlib_export_plugin_description_file(hardware_interface unitree_z1_hw_interface.xml)

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    )
install(
    FILES ${UNITREE_Z1_SDK_LIB}
    DESTINATION lib
    )
install(
    DIRECTORY include
    DESTINATION include
    )
install(
    DIRECTORY launch config urdf meshes rviz
    DESTINATION share/${PROJECT_NAME}
    )

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_REQUIRED_COMPONENTS})
ament_package()
