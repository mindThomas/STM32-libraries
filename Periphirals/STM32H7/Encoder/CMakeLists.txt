set(LIBRARY_NAME STM32H7_Encoder)
set(LIBRARY_DEPENDENCIES
    HAL::STM32::H7::M7::RCC
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::TIM
    HAL::STM32::H7::M7::CORTEX
    CMSIS::STM32::H7::M7)
set(LIBRARY_SOURCES Encoder.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines STM32H7_ENCODER_USE_DEBUG when option is enabled
add_library_option(${LIBRARY_NAME} DEBUG STM32_DEBUG)
