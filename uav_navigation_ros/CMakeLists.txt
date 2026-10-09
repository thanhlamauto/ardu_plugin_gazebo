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
find_package(mavros_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tinyxml2_vendor REQUIRED)
find_package(TinyXML2 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

add_library(depth_anything_onnx STATIC src/depth_anything_onnx.cpp)
target_compile_features(depth_anything_onnx PUBLIC cxx_std_17)
target_include_directories(depth_anything_onnx PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS})
target_link_libraries(depth_anything_onnx PUBLIC
  uav_navigation_core::uav_navigation_core ${OpenCV_LIBS})

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

add_executable(monocular_depth_node src/monocular_depth_node.cpp)
target_compile_features(monocular_depth_node PRIVATE cxx_std_17)
target_link_libraries(monocular_depth_node depth_anything_onnx)
ament_target_dependencies(monocular_depth_node
  rclcpp sensor_msgs diagnostic_msgs)
set_target_properties(monocular_depth_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

add_library(autopilot_adapter_logic STATIC src/autopilot_adapter.cpp)
target_compile_features(autopilot_adapter_logic PUBLIC cxx_std_17)
target_include_directories(autopilot_adapter_logic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(autopilot_adapter_node src/autopilot_adapter_node.cpp)
target_compile_features(autopilot_adapter_node PRIVATE cxx_std_17)
target_link_libraries(autopilot_adapter_node autopilot_adapter_logic)
ament_target_dependencies(autopilot_adapter_node
  rclcpp geometry_msgs diagnostic_msgs mavros_msgs)
set_target_properties(autopilot_adapter_node PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)

# RoboStack's macOS message libraries are Python extension modules with
# dynamic Python symbols. Match the Python ABI provided by the active ROS
# environment (Jazzy and Humble may use different Python versions).
if(APPLE)
  find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
  target_link_libraries(global_planner_node Python3::Python)
  target_link_libraries(local_navigation_node Python3::Python)
  target_link_libraries(monocular_depth_node Python3::Python)
  target_link_libraries(autopilot_adapter_node Python3::Python)
endif()

add_executable(depth_anything_onnx_probe test/depth_anything_onnx_probe.cpp)
target_link_libraries(depth_anything_onnx_probe depth_anything_onnx)
set_target_properties(depth_anything_onnx_probe PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE INSTALL_RPATH_USE_LINK_PATH TRUE)

add_executable(depth_anything_onnx_stream test/depth_anything_onnx_stream.cpp)
target_link_libraries(depth_anything_onnx_stream depth_anything_onnx)
set_target_properties(depth_anything_onnx_stream PROPERTIES
  BUILD_RPATH_USE_ORIGIN TRUE INSTALL_RPATH_USE_LINK_PATH TRUE)

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

  add_executable(test_autopilot_adapter test/test_autopilot_adapter.cpp)
  target_link_libraries(test_autopilot_adapter PRIVATE autopilot_adapter_logic)
  add_test(NAME autopilot_adapter_tests COMMAND test_autopilot_adapter)
endif()

install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(TARGETS sdf_static_map_loader autopilot_adapter_logic depth_anything_onnx
  global_planner_node local_navigation_node monocular_depth_node autopilot_adapter_node
  depth_anything_onnx_probe depth_anything_onnx_stream
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
