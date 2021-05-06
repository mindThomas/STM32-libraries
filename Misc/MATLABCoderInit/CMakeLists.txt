set(LIBRARY_NAME Misc_MATLABCoderInit)
set(LIBRARY_DEPENDENCIES)
set(LIBRARY_SOURCES MATLABCoderInit.cpp rtGetInf.cpp rtGetNaN.cpp rt_nonfinite.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})
