set(LIBRARY_NAME STM32_LSPC)
set(LIBRARY_DEPENDENCIES)
set(PERIPHERAL_DEPENDENCIES IO)
set(LIBRARY_SOURCES LSPC.cpp Packet.hpp Serializable.hpp SocketBase.hpp)

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

# Defines STM32_LSPC_USE_DEBUG when option is enabled
add_library_option(${LIBRARY_NAME} DEBUG STM32_Debug)

# Define the type of LSPC interface
# Select using e.g. 'enable_option(your_target STM32_LSPC "UART")'
add_library_option(${LIBRARY_NAME} UART $<TARGET_PROPERTY:DEVICE_FAMILY>_UART)
add_library_option(${LIBRARY_NAME} USB $<TARGET_PROPERTY:DEVICE_FAMILY>_USBCDC)
