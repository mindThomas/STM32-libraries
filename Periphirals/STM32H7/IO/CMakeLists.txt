set(LIBRARY_NAME           STM32H7_IO
)
set(LIBRARY_DEPENDENCIES   HAL::STM32::H7::M7::RCC
                           HAL::STM32::H7::M7::GPIO
                           HAL::STM32::H7::M7::CORTEX
                           CMSIS::STM32::H7::M7
)
set(LIBRARY_SOURCES        IO.cpp
)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Add device-specific CMSIS library
#target_link_libraries(${LIBRARY_NAME} INTERFACE
#        $<IF:$<BOOL:$<TARGET_PROPERTY:DEVICE_CORE>>,CMSIS::STM32::$<TARGET_PROPERTY:DEVICE_NAME>::$<TARGET_PROPERTY:DEVICE_CORE>,CMSIS::STM32::$<TARGET_PROPERTY:DEVICE_NAME>>
#)

# Defines STM32H7_IO_USE_DEBUG when option is enabled
add_library_option(${LIBRARY_NAME} DEBUG STM32_DEBUG)

