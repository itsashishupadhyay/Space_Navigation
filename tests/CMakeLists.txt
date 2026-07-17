include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.16.0.zip
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(limbnav_tests
  test_geometry.cpp
  test_io.cpp
  test_preprocess.cpp)
target_link_libraries(limbnav_tests PRIVATE limbnav gtest_main)
target_compile_definitions(limbnav_tests
  PRIVATE LIMBNAV_REPO_ROOT="${PROJECT_SOURCE_DIR}")

include(GoogleTest)
gtest_discover_tests(limbnav_tests)

# tests/synthetic/: G0 renderer + goldens land in P1 (§8).
