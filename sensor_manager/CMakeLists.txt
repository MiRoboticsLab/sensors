cmake_minimum_required(VERSION 3.8)
project(sensor_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(manager_base REQUIRED)
find_package(protocol REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cyberdog_system REQUIRED)
find_package(cyberdog_gps REQUIRED)
find_package(bream_vendor REQUIRED)
find_package(cyberdog_ultrasonic REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(cyberdog_tof REQUIRED)



include_directories(include)

add_executable(${PROJECT_NAME}
  src/sensor_manager.cpp
  src/main.cpp)


target_link_libraries(${PROJECT_NAME}
  ${cyberdog_log_LIBRARIES}
) 

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  protocol
  manager_base
  cyberdog_system
  pluginlib
  cyberdog_gps
  cyberdog_ultrasonic
  cyberdog_tof)

install( TARGETS
  ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
