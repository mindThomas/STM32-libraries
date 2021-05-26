set(LIBRARY_NAME LSPC)
set(LIBRARY_DEPENDENCIES)
set(LIBRARY_SOURCES LSPC.cpp Packet.hpp Serializable.hpp SocketBase.hpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines LSPC_USE_DEBUG when option is enabled
add_library_option_with_dependency(${LIBRARY_NAME} DEBUG Debug)

# Define the type of LSPC interface
# Select using e.g. 'enable_option(your_target LSPC_ "UART")'
add_library_option_with_dependency(${LIBRARY_NAME} UART $<TARGET_PROPERTY:DEVICE_FAMILY>_UART)
add_library_option_with_dependency(${LIBRARY_NAME} USB $<TARGET_PROPERTY:DEVICE_FAMILY>_USBCDC)

# Define LSPC_TX_QUEUE_LENGTH as configurable option
add_configurable_library_option(${LIBRARY_NAME} TX_QUEUE_LENGTH)