cmake_minimum_required(VERSION 3.8)
project(cyberdog_ultrasonic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(params REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cyberdog_common REQUIRED)
find_package(protocol REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(cyberdog_embed_protocol REQUIRED)
find_package(cyberdog_system REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/ultrasonic_plugin.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME}
    ${cyberdog_log_LIBRARIES}
)

# define project name to get relative share path
target_compile_definitions(${PROJECT_NAME} PRIVATE "PACKAGE_NAME=\"${PROJECT_NAME}\"")
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

ament_target_dependencies(
  ${PROJECT_NAME}
  pluginlib
  params
  rclcpp
  cyberdog_common
  ament_index_cpp
  protocol
  cyberdog_embed_protocol
)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include/
)


install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_package()
