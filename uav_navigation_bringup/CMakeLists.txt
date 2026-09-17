cmake_minimum_required(VERSION 3.16)
project(uav_navigation_bringup VERSION 0.1.0)

find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/gz_sim_entrypoint DESTINATION bin)
ament_package()
