cmake_minimum_required(VERSION 3.16)
project(uav_navigation_ros VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
find_package(ament_cmake REQUIRED)
find_package(uav_navigation_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(tinyxml2 REQUIRED CONFIG)

add_library(sdf_static_map_loader STATIC src/sdf_static_map_loader.cpp)
target_compile_features(sdf_static_map_loader PUBLIC cxx_std_17)
target_include_directories(sdf_static_map_loader PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sdf_static_map_loader PUBLIC
  uav_navigation_core::uav_navigation_core tinyxml2::tinyxml2)

add_executable(global_planner_node src/global_planner_node.cpp)
target_compile_features(global_planner_node PRIVATE cxx_std_17)
target_link_libraries(global_planner_node
  sdf_static_map_loader uav_navigation_core::uav_navigation_core)
ament_target_dependencies(global_planner_node
  rclcpp geometry_msgs nav_msgs diagnostic_msgs)
set_target_properties(global_planner_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(local_navigation_node src/local_navigation_node.cpp)
target_compile_features(local_navigation_node PRIVATE cxx_std_17)
target_link_libraries(local_navigation_node
  uav_navigation_core::uav_navigation_core)
ament_target_dependencies(local_navigation_node
  rclcpp geometry_msgs nav_msgs sensor_msgs visualization_msgs diagnostic_msgs tf2_ros)
set_target_properties(local_navigation_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# RoboStack's macOS message libraries are Python extension modules with
# dynamic Python symbols. Native Ubuntu ROS 2 does not need this workaround.
if(APPLE)
  find_library(ROS_PYTHON_RUNTIME NAMES python3.12 PATH_SUFFIXES lib REQUIRED)
  target_link_libraries(global_planner_node ${ROS_PYTHON_RUNTIME})
  target_link_libraries(local_navigation_node ${ROS_PYTHON_RUNTIME})
endif()

if(BUILD_TESTING)
  add_executable(test_sdf_static_map_loader test/test_sdf_static_map_loader.cpp)
  target_link_libraries(test_sdf_static_map_loader PRIVATE sdf_static_map_loader)
  if(APPLE)
    set_property(TARGET test_sdf_static_map_loader APPEND PROPERTY BUILD_RPATH
      "$<TARGET_FILE_DIR:tinyxml2::tinyxml2>")
  endif()
  target_compile_definitions(test_sdf_static_map_loader PRIVATE
    TEST_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/../worlds/iris_mppi_yard_runup60.sdf"
    TEST_CYLINDER_WORLD="${CMAKE_CURRENT_SOURCE_DIR}/../worlds/iris_mppi_slalom.sdf")
  add_test(NAME sdf_static_map_loader_tests COMMAND test_sdf_static_map_loader)
endif()

install(DIRECTORY include/ DESTINATION include)
install(TARGETS sdf_static_map_loader global_planner_node local_navigation_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/safe_twist_to_mavlink.py
  DESTINATION lib/${PROJECT_NAME})
ament_package()
