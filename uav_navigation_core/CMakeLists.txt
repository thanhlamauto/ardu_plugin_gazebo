cmake_minimum_required(VERSION 3.16)
project(uav_navigation_core VERSION 0.1.0 LANGUAGES CXX)

include(CTest)
include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(${PROJECT_NAME} SHARED
  src/astar_planner.cpp
  src/command_conditioner.cpp
  src/cost_grid.cpp
  src/trajectory_safety_checker.cpp)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(${PROJECT_NAME} PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR})

if(BUILD_TESTING)
  add_executable(test_astar test/test_astar.cpp)
  target_link_libraries(test_astar PRIVATE ${PROJECT_NAME})
  add_test(NAME astar_core_tests COMMAND test_astar)

  add_executable(test_command_conditioner test/test_command_conditioner.cpp)
  target_link_libraries(test_command_conditioner PRIVATE ${PROJECT_NAME})
  target_compile_definitions(test_command_conditioner PRIVATE
    UAV_NAVIGATION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  add_test(NAME command_conditioner_tests COMMAND test_command_conditioner)

  add_executable(test_trajectory_safety test/test_trajectory_safety.cpp)
  target_link_libraries(test_trajectory_safety PRIVATE ${PROJECT_NAME})
  target_compile_definitions(test_trajectory_safety PRIVATE
    UAV_NAVIGATION_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  add_test(NAME trajectory_safety_tests COMMAND test_trajectory_safety)
endif()

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ${PROJECT_NAME}Targets
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

configure_package_config_file(
  cmake/${PROJECT_NAME}Config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME})

# Export through ament when ROS 2 is available. The library itself remains
# independent of ROS, Gazebo and SDF.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
  ament_export_include_directories(include)
  ament_package()
endif()
