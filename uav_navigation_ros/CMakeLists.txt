cmake_minimum_required(VERSION 3.16)
project(uav_navigation_ros VERSION 0.1.0 LANGUAGES CXX)

find_package(ament_cmake REQUIRED)

# Architecture checkpoint only. C++ nodes are added after interface review.
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_package()
