cmake_minimum_required(VERSION 3.8)
project(ros_template_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

# Create library for testing
add_library(${PROJECT_NAME} src/template_node.cpp)

# Include directories for library
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Target compile features
target_compile_features(${PROJECT_NAME} PUBLIC c_std_99 cxx_std_17)

# Link dependencies to library
ament_target_dependencies(
  ${PROJECT_NAME}
  "rclcpp"
  "std_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "diagnostic_msgs"
)

# Create executable
add_executable(template_node src/main.cpp)
target_link_libraries(template_node ${PROJECT_NAME})

# Install targets
install(TARGETS template_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test test/test_template_node.cpp)
  if(TARGET ${PROJECT_NAME}_test)
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_test
      "rclcpp"
      "std_msgs"
      "geometry_msgs"
      "sensor_msgs"
      "diagnostic_msgs"
    )
  endif()

  # Integration tests
  ament_add_gtest(${PROJECT_NAME}_integration_test test/test_integration.cpp)
  if(TARGET ${PROJECT_NAME}_integration_test)
    target_link_libraries(${PROJECT_NAME}_integration_test ${PROJECT_NAME})
    ament_target_dependencies(${PROJECT_NAME}_integration_test
      "rclcpp"
      "std_msgs"
      "geometry_msgs"
      "sensor_msgs"
      "diagnostic_msgs"
    )
  endif()
endif()

ament_package()