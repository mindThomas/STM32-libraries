set(LIBRARY_NAME Matrix)
set(LIBRARY_DEPENDENCIES MATLABCoderInit)
file(GLOB_RECURSE LIBRARY_SOURCES *.cpp) # find all cpp files

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines MATRIX_USE_DEBUG when option is enabled
add_library_option(${LIBRARY_NAME} DEBUG Debug)