set(LIBRARY_NAME STM32H7_USBCDC)
set(LIBRARY_DEPENDENCIES
    HAL::STM32::H7::M7::LL_USB
    HAL::STM32::H7::M7::PCD
    HAL::STM32::H7::M7::PCDEx
    HAL::STM32::H7::M7::PWREx
    HAL::STM32::H7::M7::RCC
    HAL::STM32::H7::M7::GPIO
    HAL::STM32::H7::M7::CORTEX
    CMSIS::STM32::H7::M7)
set(LIBRARY_SOURCES
    USBCDC.cpp
    usbd_cdc_if.c
    usbd_conf.c
    usbd_desc.c
    STM32_USB_Device_Library/Class/CDC/Src/usbd_cdc.c
    STM32_USB_Device_Library/Core/Src/usbd_core.c
    STM32_USB_Device_Library/Core/Src/usbd_ctlreq.c
    STM32_USB_Device_Library/Core/Src/usbd_ioreq.c)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME}
                      INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines STM32H7_USBCDC_USE_DEBUG when option is enabled
add_library_option(${LIBRARY_NAME} DEBUG STM32_DEBUG)

# Define STM32H7_USBCDC_DEVICE_NAME as configurable option
add_configurable_library_option(${LIBRARY_NAME} DEVICE_NAME)
