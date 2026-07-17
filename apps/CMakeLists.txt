add_executable(measure_limb measure_limb.cpp)
target_link_libraries(measure_limb PRIVATE limbnav opencv_imgcodecs)

add_executable(batch_eval batch_eval.cpp)
target_link_libraries(batch_eval PRIVATE limbnav opencv_imgcodecs)
