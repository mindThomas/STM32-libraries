# Serves as a replacement of the SysTick by configuring TIM16 as a high-frequency SysTick
# which integrates with the HAL_Delay and Tick framework

set(LIBRARY_NAME STM32H7_PrecisionSysTick)
set(LIBRARY_DEPENDENCIES
    HAL::STM32::H7::M7::TIM
    HAL::STM32::H7::M7::TIMEx
    HAL::STM32::H7::M7::CORTEX
    CMSIS::STM32::H7::M7)
set(LIBRARY_SOURCES stm32h7xx_hal_timebase_tim.c)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Define STM32H7_PRECISION_SYSTICK_FREQUENCY as configurable option
add_configurable_library_option(${LIBRARY_NAME} FREQUENCY)
