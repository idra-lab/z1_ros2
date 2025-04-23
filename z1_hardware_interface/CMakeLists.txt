cmake_minimum_required(VERSION 3.8)
project(z1_hardware_interface)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3   REQUIRED)
find_package(fmt      REQUIRED)
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

#  _   _       _ _                   ____       _
# | | | |_ __ (_) |_ _ __ ___  ___  / ___|  ___| |_ _   _ _ __
# | | | | '_ \| | __| '__/ _ \/ _ \ \___ \ / _ \ __| | | | '_ \
# | |_| | | | | | |_| | |  __/  __/  ___) |  __/ |_| |_| | |_) |
#  \___/|_| |_|_|\__|_|  \___|\___| |____/ \___|\__|\__,_| .__/
#                                                        |_|
# Import unitree controller and SDK and setup targets

include(FetchContent)
FetchContent_Declare(
    z1_controller
    URL https://github.com/unitreerobotics/z1_controller/archive/refs/heads/master.zip
)
FetchContent_MakeAvailable(z1_controller)

FetchContent_Declare(
    z1_sdk
    URL https://github.com/unitreerobotics/z1_sdk/archive/refs/heads/master.zip
)
FetchContent_GetProperties(z1_sdk)
if(NOT z1_sdk_FOUND)
    message(STATUS "z1_sdk not found, fetching...")
    FetchContent_Populate(z1_sdk)
endif()

set(Z1_CONTROLLER_PATH ${z1_controller_SOURCE_DIR})
set(Z1_SDK_PATH        ${z1_sdk_SOURCE_DIR})
set(Z1_CONTROLLER_LIB  ${Z1_CONTROLLER_PATH}/lib/libZ1_${CMAKE_SYSTEM_PROCESSOR}.so)
set(Z1_SDK_LIB         ${Z1_SDK_PATH}/lib/libZ1_SDK_${CMAKE_SYSTEM_PROCESSOR}.so)


add_library(
    unitree::z1_controller 
    INTERFACE IMPORTED 
    ${Z1_CONTROLLER_LIB} 
)
target_include_directories(
    unitree::z1_controller
    INTERFACE
    ${Z1_CONTROLLER_PATH}/include
)

add_library(
    unitree::z1_sdk
    INTERFACE IMPORTED
    ${Z1_SDK_LIB}
)
target_include_directories(
    unitree::z1_sdk
    INTERFACE
    ${Z1_SDK_PATH}/include
)


#  _   _               _                        
# | | | | __ _ _ __ __| |_      ____ _ _ __ ___ 
# | |_| |/ _` | '__/ _` \ \ /\ / / _` | '__/ _ \
# |  _  | (_| | | | (_| |\ V  V / (_| | | |  __/
# |_| |_|\__,_|_|  \__,_| \_/\_/ \__,_|_|  \___|
#                                               
#  ___       _             __                
# |_ _|_ __ | |_ ___ _ __ / _| __ _  ___ ___ 
#  | || '_ \| __/ _ \ '__| |_ / _` |/ __/ _ \
#  | || | | | ||  __/ |  |  _| (_| | (_|  __/
# |___|_| |_|\__\___|_|  |_|  \__,_|\___\___|
#                                            
# Hardware interface target

add_library(
    ${PROJECT_NAME}
    SHARED
    src/z1_hardware_interface.cpp
)
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(
    ${PROJECT_NAME} 
    PRIVATE
    Eigen3::Eigen
    fmt::fmt
    PUBLIC
    unitree::z1_sdk
    ${Z1_SDK_LIB}
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

pluginlib_export_plugin_description_file(hardware_interface z1_hardware_interface.xml)

# Target install
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(
    DIRECTORY include
    DESTINATION include
)

# SDK and libraries
install(
    FILES       ${Z1_CONTROLLER_LIB} ${Z1_SDK_LIB}
    DESTINATION lib
)
install(
    FILES      ${CMAKE_CURRENT_BINARY_DIR}/z1_ctrl
    DESTINATION share/${PROJECT_NAME}/controller
)
install(
    DIRECTORY   config scripts
    DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_REQUIRED_COMPONENTS})

ament_package()
