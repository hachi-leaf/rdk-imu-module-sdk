# Copyright (c) 2026 Leaf. D-Robotics.
# SPDX-License-Identifier: MIT
cmake_minimum_required(VERSION 3.8)
project(rdk_imu_module)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

set(RDKIMU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../core")
set(RDKIMU_INCLUDE_DIR "${RDKIMU_ROOT}/include")

find_library(RDKIMU_LIBRARY rdkimu
  PATHS "${RDKIMU_ROOT}/lib"
  NO_DEFAULT_PATH
  REQUIRED
)

if(NOT EXISTS "${RDKIMU_INCLUDE_DIR}/rdkimu.h")
  message(FATAL_ERROR "rdkimu.h not found at ${RDKIMU_INCLUDE_DIR}")
endif()

add_executable(rdk_imu_node src/rdk_imu_node.cpp)

target_include_directories(rdk_imu_node PRIVATE ${RDKIMU_INCLUDE_DIR})
ament_target_dependencies(rdk_imu_node rclcpp sensor_msgs std_srvs)
target_link_libraries(rdk_imu_node ${RDKIMU_LIBRARY})

find_package(Threads REQUIRED)
target_link_libraries(rdk_imu_node Threads::Threads)

set_target_properties(rdk_imu_node PROPERTIES
  BUILD_RPATH "\$ORIGIN"
  INSTALL_RPATH "\$ORIGIN"
)

install(TARGETS rdk_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${RDKIMU_ROOT}/lib/librdkimu.so.1.0.0
  DESTINATION lib/${PROJECT_NAME}
  RENAME librdkimu.so.1.0.0
)
install(FILES ${RDKIMU_ROOT}/lib/librdkimu.so.1.0.0
  DESTINATION lib/${PROJECT_NAME}
  RENAME librdkimu.so.1
)

install(FILES ${RDKIMU_ROOT}/lib/librdkimu.so.1.0.0
  DESTINATION lib/${PROJECT_NAME}
  RENAME librdkimu.so
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()