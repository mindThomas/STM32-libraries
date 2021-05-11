set(LIBRARY_NAME CPULoad)
set(LIBRARY_DEPENDENCIES LSPC)
set(LIBRARY_SOURCES CPULoad.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Enable verbose output with 'enable_option(your_target CPULoad VERBOSE)'
add_library_option(${LIBRARY_NAME} VERBOSE)

# OBS. Requires the printf with Floating point support.
# For the Newlib nano library (STM32::Nano) this enabled with (taking up approximately 8 KB):
# enable_option(firmware STM32::Nano FLOAT)