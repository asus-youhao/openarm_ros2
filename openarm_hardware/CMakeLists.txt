# Copyright 2025 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(openarm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(dynamixel_sdk REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(urdf REQUIRED)
find_package(yaml-cpp REQUIRED)

# Find openarm_can library
find_package(OpenArmCAN REQUIRED)

# Find LinkerHand SDK library (for O6 hand)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(LIB_SUBDIR "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    set(LIB_SUBDIR "aarch64")
else()
    message(WARNING "Unknown architecture for LinkerHand SDK, defaulting to x86_64")
    set(LIB_SUBDIR "x86_64")
endif()

find_library(LINKER_HAND_LIB
    NAMES linkerhand_cpp_sdk linkerhand_cpp
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../linkerhand-cpp-ros2/src/linker_hand_cpp_ros2/lib/${LIB_SUBDIR}
        /usr/local/lib/linkerhand-cpp-sdk/${LIB_SUBDIR}
        /usr/lib/linkerhand-cpp-sdk/${LIB_SUBDIR}
        ${CMAKE_INSTALL_PREFIX}/lib/linkerhand-cpp-sdk/${LIB_SUBDIR}
    NO_DEFAULT_PATH
)

set(LINKER_HAND_INCLUDE_DIR
    ${CMAKE_CURRENT_SOURCE_DIR}/../linkerhand-cpp-ros2/src/linker_hand_cpp_ros2/include
    /usr/local/include/linkerhand-cpp-sdk
    /usr/include/linkerhand-cpp-sdk
    ${CMAKE_INSTALL_PREFIX}/include/linkerhand-cpp-sdk
)

if(LINKER_HAND_LIB)
    message(STATUS "Found linkerhand_cpp_sdk library: ${LINKER_HAND_LIB}")
    message(STATUS "Found LinkerHand headers: ${LINKER_HAND_INCLUDE_DIR}")
    set(HAVE_LINKER_HAND TRUE)
else()
    message(WARNING "linkerhand_cpp_sdk library not found! O6HandHardware will not be built.")
    set(HAVE_LINKER_HAND FALSE)
endif()

add_library(${PROJECT_NAME} SHARED
  src/v10_simple_hardware.cpp
  src/leap_hand_hardware.cpp
)

# Conditionally add O6 hand hardware if SDK is found
if(HAVE_LINKER_HAND)
  target_sources(${PROJECT_NAME} PRIVATE src/o6_hand_hardware.cpp)
  target_include_directories(${PROJECT_NAME} PRIVATE ${LINKER_HAND_INCLUDE_DIR})
  target_link_libraries(${PROJECT_NAME} ${LINKER_HAND_LIB} pthread)
endif()

target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
)

target_link_libraries(${PROJECT_NAME}
  OpenArmCAN::openarm_can
  yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ament_index_cpp
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  dynamixel_sdk
  kdl_parser
  orocos_kdl
  urdf
)

pluginlib_export_plugin_description_file(hardware_interface openarm_hardware.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
  )

install(DIRECTORY include/
DESTINATION include
)

# Install config files (parameters.yaml, etc.)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# Install Python scripts
install(PROGRAMS
  scripts/test_leap_serial_port.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
)

ament_package()
