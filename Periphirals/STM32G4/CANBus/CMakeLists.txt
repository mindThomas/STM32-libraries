set(LIBRARY_NAME STM32G4_CANBus)
set(LIBRARY_DEPENDENCIES
    HAL::STM32::G4::FDCAN
    HAL::STM32::G4::RCC
    HAL::STM32::G4::GPIO
    HAL::STM32::G4::CORTEX
    CMSIS::STM32::G4)
set(LIBRARY_SOURCES CANBus.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines STM32G4_CANBUS_USE_DEBUG when option is enabled
add_library_option_with_dependency(${LIBRARY_NAME} DEBUG Debug)
