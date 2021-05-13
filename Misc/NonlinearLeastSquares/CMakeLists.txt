set(LIBRARY_NAME NonlinearLeastSquares)
set(LIBRARY_DEPENDENCIES Matrix MATLABCoderInit)
set(LIBRARY_SOURCES
        NonlinearLeastSquares.cpp
        exp.cpp
        fitExponentialDecay2.cpp
        fitExponentialDecay2_emxAPI.cpp
        fitExponentialDecay2_emxutil.cpp)

message("       -- ${LIBRARY_NAME}")

add_library(${LIBRARY_NAME} INTERFACE)
target_sources(${LIBRARY_NAME} INTERFACE ${LIBRARY_SOURCES})
target_link_libraries(${LIBRARY_NAME} INTERFACE ${LIBRARY_DEPENDENCIES})
get_filename_component(INCLUDE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
target_include_directories(${LIBRARY_NAME} INTERFACE ${INCLUDE_PATH})

# Defines NONLINEARLEASTSQUARES_USE_DEBUG when option is enabled
add_library_option_with_dependency(${LIBRARY_NAME} DEBUG Debug)