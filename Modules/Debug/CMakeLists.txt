set(LIBRARY_NAME STM32_DEBUG)
set(LIBRARY_DEPENDENCIES)
set(PERIPHERAL_DEPENDENCIES IO)
set(LIBRARY_SOURCES Debug.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Add device-specific periphiral libraries, e.g. STM32F4_IO
foreach(dependency ${PERIPHERAL_DEPENDENCIES})
    target_link_libraries(${LIBRARY_NAME}
                          INTERFACE $<TARGET_PROPERTY:DEVICE_FAMILY>_${dependency})
endforeach()
