cmake_minimum_required(VERSION 3.21)
project(limbnav VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(LIMBNAV_BUILD_TESTS "Build gtest suite" ON)
option(LIMBNAV_ENABLE_YOLO_GATE "Enable ONNX detection gate (needs opencv dnn)" ON)
option(LIMBNAV_WITH_CSPICE "Link CSPICE for onboard geometry (Phase 2)" OFF)

# Metrology: determinism and IEEE compliance are non-negotiable (§4.2).
if(CMAKE_CXX_FLAGS MATCHES "ffast-math")
  message(FATAL_ERROR "-ffast-math is forbidden in limbnav (CLAUDE.md §4.2)")
endif()

find_package(OpenCV REQUIRED COMPONENTS core imgproc dnn)

execute_process(
  COMMAND git rev-parse --short=12 HEAD
  WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
  OUTPUT_VARIABLE LIMBNAV_GIT_SHA
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET)
if(NOT LIMBNAV_GIT_SHA)
  set(LIMBNAV_GIT_SHA "unknown")
endif()

# Shared usage requirements for every module.
add_library(limbnav_settings INTERFACE)
target_include_directories(limbnav_settings INTERFACE ${PROJECT_SOURCE_DIR}/include)
target_compile_options(limbnav_settings INTERFACE -Wall -Wextra -Wpedantic)

add_subdirectory(src)
add_subdirectory(apps)

if(LIMBNAV_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()
