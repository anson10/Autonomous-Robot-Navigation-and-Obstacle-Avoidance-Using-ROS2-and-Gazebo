cmake_minimum_required(VERSION 3.8)
project(my_robot_navigation_cpp)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)

# Planner core: plain C++, no ROS. Shared by the node, the tests and the Python module.
add_library(avoidance_planner STATIC
  src/avoidance_planner.cpp
  src/waypoints.cpp)
target_include_directories(avoidance_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
set_target_properties(avoidance_planner PROPERTIES POSITION_INDEPENDENT_CODE ON)

add_executable(obstacle_avoidance_node src/obstacle_avoidance_node.cpp)
target_link_libraries(obstacle_avoidance_node avoidance_planner)
ament_target_dependencies(obstacle_avoidance_node
  rclcpp geometry_msgs nav_msgs rcl_interfaces sensor_msgs std_msgs std_srvs
  visualization_msgs)

pybind11_add_module(_planner src/python_bindings.cpp)
target_link_libraries(_planner PRIVATE avoidance_planner)

install(TARGETS obstacle_avoidance_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
ament_python_install_package(${PROJECT_NAME})
install(TARGETS _planner DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  # Files carry SPDX MIT headers, which ament_copyright does not recognise.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_avoidance_planner test/test_avoidance_planner.cpp)
  target_link_libraries(test_avoidance_planner avoidance_planner)

  # Parity with the Python reference planner, via the pybind11 module.
  ament_add_pytest_test(test_parity test/test_parity.py
    PYTHON_EXECUTABLE "${Python3_EXECUTABLE}"
    TIMEOUT 300)
endif()

ament_package()
