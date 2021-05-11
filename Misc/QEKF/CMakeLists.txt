set(LIBRARY_NAME QEKF)
set(LIBRARY_DEPENDENCIES MathLib Parameters)
set(PERIPHERAL_DEPENDENCIES Timer)
set(LIBRARY_SOURCES QEKF.cpp
        MATLABCoder/eye.cpp
        MATLABCoder/mrdivide.cpp
        MATLABCoder/norm.cpp
        MATLABCoder/QEKF_coder.cpp
        MATLABCoder/QEKF_initialize.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Add device-specific periphiral libraries, e.g. STM32F4_Timer
foreach(dependency ${PERIPHERAL_DEPENDENCIES})
    target_link_libraries(
            ${LIBRARY_NAME}
            INTERFACE $<TARGET_PROPERTY:DEVICE_FAMILY>_${dependency})
endforeach()