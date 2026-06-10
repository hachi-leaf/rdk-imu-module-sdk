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

# 定位 rdkimu 库和头文件
set(RDKIMU_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../core")
set(RDKIMU_INCLUDE_DIR "${RDKIMU_ROOT}/include")
set(RDKIMU_LIBRARY "${RDKIMU_ROOT}/lib/librdkimu.so")

if(NOT EXISTS "${RDKIMU_INCLUDE_DIR}/rdkimu.h")
  message(FATAL_ERROR "rdkimu.h not found at ${RDKIMU_INCLUDE_DIR}")
endif()
if(NOT EXISTS "${RDKIMU_LIBRARY}")
  message(FATAL_ERROR "librdkimu.so not found at ${RDKIMU_LIBRARY}")
endif()

include_directories(${RDKIMU_INCLUDE_DIR})

add_executable(rdk_imu_node src/rdk_imu_node.cpp)

# 自动处理所有 ROS 2 依赖（rclcpp, sensor_msgs, std_srvs）
ament_target_dependencies(rdk_imu_node rclcpp sensor_msgs std_srvs)

# 链接外部库
target_link_libraries(rdk_imu_node ${RDKIMU_LIBRARY})
find_package(Threads REQUIRED)
target_link_libraries(rdk_imu_node Threads::Threads)

install(TARGETS rdk_imu_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()