# Serves as a replacement of the SysTick by configuring TIM15 as a high-frequency SysTick
# which integrates with the HAL_Delay and Tick framework
# Configured (by default) to run at 100 KHz, thereby giving higher precision ticks at 10 us precision
# Default frequency can be changed with

set(LIBRARY_NAME STM32G4_PrecisionSysTick)
set(LIBRARY_DEPENDENCIES HAL::STM32::G4::TIM HAL::STM32::G4::CORTEX CMSIS::STM32::G4)
set(LIBRARY_SOURCES PrecisionSysTick.c)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

target_compile_definitions(${LIBRARY_NAME} INTERFACE USE_PRECISION_SYSTICK)

# Define STM32G4_PRECISIONSYSTICK_FREQUENCY as configurable option
add_configurable_library_option(${LIBRARY_NAME} FREQUENCY)
