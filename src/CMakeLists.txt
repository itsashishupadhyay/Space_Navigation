add_subdirectory(io)
add_subdirectory(geometry)
add_subdirectory(preprocess)
add_subdirectory(limb)
add_subdirectory(fit)

add_library(limbnav STATIC
  $<TARGET_OBJECTS:limbnav_io>
  $<TARGET_OBJECTS:limbnav_geometry>
  $<TARGET_OBJECTS:limbnav_preprocess>
  $<TARGET_OBJECTS:limbnav_limb>
  $<TARGET_OBJECTS:limbnav_fit>)
target_link_libraries(limbnav PUBLIC limbnav_settings opencv_core opencv_imgproc)
