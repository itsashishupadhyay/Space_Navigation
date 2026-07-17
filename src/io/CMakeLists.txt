add_library(limbnav_io OBJECT io.cpp)
target_link_libraries(limbnav_io PRIVATE limbnav_settings)
target_compile_definitions(limbnav_io PRIVATE LIMBNAV_GIT_SHA="${LIMBNAV_GIT_SHA}")
