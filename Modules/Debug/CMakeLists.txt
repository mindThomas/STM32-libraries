set(LIBRARY_NAME Debug)
set(LIBRARY_DEPENDENCIES)
set(PERIPHERAL_DEPENDENCIES IO)
set(LIBRARY_SOURCES Debug.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Add device-specific periphiral libraries, e.g. STM32F4_IO
foreach(dependency ${PERIPHERAL_DEPENDENCIES})
    target_link_libraries(
        ${LIBRARY_NAME}
        INTERFACE $<TARGET_PROPERTY:DEVICE_FAMILY>_${dependency})
endforeach()

# Define the type of DEBUG interface for PRINTF support
# Select using e.g. 'enable_option(your_target Debug "UART")'
add_library_option_with_dependency(${LIBRARY_NAME} UART $<TARGET_PROPERTY:DEVICE_FAMILY>_UART)
add_library_option_with_dependency(${LIBRARY_NAME} LSPC LSPC)
add_library_option(${LIBRARY_NAME} PRINTF)
add_library_option(${LIBRARY_NAME} VERBOSE)
