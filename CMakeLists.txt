cmake_minimum_required(VERSION 3.18)

project(chrono-actuators VERSION 0.1.0 LANGUAGES CXX)

option(CHRONO_ACTUATORS_ENABLE_ROS "Enable optional ROS I/O scaffold" OFF)
option(CHRONO_ACTUATORS_ENABLE_HIL "Enable optional HIL I/O scaffold" OFF)
option(CHRONO_ACTUATORS_BUILD_DEMOS "Build chrono-actuators demos" ON)
option(CHRONO_ACTUATORS_BUILD_TESTS "Build chrono-actuators tests" ON)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++20" CHRONO_ACTUATORS_HAS_CXX20)
if(CHRONO_ACTUATORS_HAS_CXX20)
  set(CMAKE_CXX_STANDARD 20)
else()
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_STANDARD LESS 20)
  message(STATUS "chrono-actuators: building with C++17 (C++20 preferred when available)")
endif()

find_package(Chrono REQUIRED CONFIG)

add_subdirectory(src)

if(CHRONO_ACTUATORS_BUILD_DEMOS)
  add_subdirectory(demos)
endif()

if(CHRONO_ACTUATORS_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
