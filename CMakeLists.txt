cmake_minimum_required(VERSION 3.8)
project(unitree_z1_hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


#  ____  ____  _  __  ____       _
# / ___||  _ \| |/ / / ___|  ___| |_ _   _ _ __
# \___ \| | | | ' /  \___ \ / _ \ __| | | | '_ \
#  ___) | |_| | . \   ___) |  __/ |_| |_| | |_) |
# |____/|____/|_|\_\ |____/ \___|\__|\__,_| .__/
#                                         |_|
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(UNITREE_Z1_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/unitree_z1_sdk)
set(UNITREE_Z1_SDK_LIB_PATH ${Z1_SKD_PATH}/lib)
set(UNITREE_Z1_SDK_INCLUDE_PATH ${Z1_SKD_PATH}/include)
set(UNITREE_Z1_SDK_LIB ${Z1_SDK_LIB_PATH}/libZ1_SKD_${CMAKE_SYSTEM_PROCESSOR}.so)



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
    ${UNITREE_Z1_HW_INTERFACE_SRC}
    )
target_include_directories(
    ${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/include
    PRIVATE
    ${UNITREE_Z1_SDK_INCLUDE_PATH}
    )
target_link_libraries(
    ${PROJECT_NAME}
    PUBLIC
    ${UNITREE_Z1_SDK_LIB}
    )



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#

install(
    TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
    )
install(
    DIRECTORY include/
    DESTINATION include
    )



pluginlib_export_plugin_description_file(hardware_interface unitree_z1_hw_interface.xml)
ament_package()
