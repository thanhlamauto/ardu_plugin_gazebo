cmake_minimum_required(VERSION 3.16)
project(openvins_gz_bridge LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(OPENVINS_ROOT "" CACHE PATH "OpenVINS source checkout")
if(NOT EXISTS "${OPENVINS_ROOT}/ov_msckf/src/core/VioManager.h")
  message(FATAL_ERROR "Set OPENVINS_ROOT to an OpenVINS checkout")
endif()

find_package(Eigen3 REQUIRED)
find_package(OpenCV 5 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem system)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GZ_TRANSPORT REQUIRED IMPORTED_TARGET gz-transport13)
pkg_check_modules(GZ_MSGS REQUIRED IMPORTED_TARGET gz-msgs10)

add_executable(openvins_gz_bridge main.cpp)
target_include_directories(openvins_gz_bridge PRIVATE
  "${OPENVINS_ROOT}/ov_msckf/src"
  "${OPENVINS_ROOT}/ov_core/src"
  "${OPENVINS_ROOT}/ov_init/src"
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS})
target_link_directories(openvins_gz_bridge PRIVATE "${OPENVINS_ROOT}/build")
target_link_libraries(openvins_gz_bridge PRIVATE ov_msckf_lib Eigen3::Eigen ${OpenCV_LIBS}
  ${Boost_LIBRARIES} PkgConfig::GZ_TRANSPORT PkgConfig::GZ_MSGS)
