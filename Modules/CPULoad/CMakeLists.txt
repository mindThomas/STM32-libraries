set(LIBRARY_NAME CPULoad)
set(LIBRARY_DEPENDENCIES LSPC)
set(LIBRARY_SOURCES CPULoad.cpp freertos.c)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

target_compile_definitions(${LIBRARY_NAME} INTERFACE
        configENABLE_BACKWARD_COMPATIBILITY=1
        configGENERATE_RUN_TIME_STATS=1
        configUSE_TRACE_FACILITY=1
        configUSE_STATS_FORMATTING_FUNCTIONS=1
        )

# Enable verbose output with 'enable_option(your_target CPULoad VERBOSE)'
add_library_option(${LIBRARY_NAME} VERBOSE)

# OBS. This CPULoad library requires the printf with Floating point support.
# To enable floating point support with the Newlib nano library (STM32::Nano) use:
# enable_option(firmware STM32::Nano FLOAT)
# This will increase the binary by approximately 8 KB