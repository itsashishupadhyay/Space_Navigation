include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.16.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(limbnav_tests
  synthetic/renderer.cpp
  test_geometry.cpp
  test_io.cpp
  test_preprocess.cpp
  test_localizers.cpp
  test_g0.cpp)
target_link_libraries(limbnav_tests PRIVATE limbnav gtest_main)
target_include_directories(limbnav_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(limbnav_tests
  PRIVATE LIMBNAV_REPO_ROOT="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(limbnav_tests PROPERTIES TIMEOUT 600)
