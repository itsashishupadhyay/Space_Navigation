add_library(limbnav_fit OBJECT fit.cpp)
target_link_libraries(limbnav_fit PRIVATE limbnav_settings opencv_core opencv_imgproc)
