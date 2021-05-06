# Copyright (C) 2021- Thomas Jespersen, TKJ Electronics. All rights reserved.
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the MIT License
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the MIT License for further details.
#
# Contact information
# ------------------------------------------
# Thomas Jespersen, TKJ Electronics
# Web      :  http://www.tkjelectronics.dk
# e-mail   :  thomasj@tkjelectronics.dk
# ------------------------------------------
#project(stm32-libraries C CXX ASM)

# Add support for C++11, C++14 or C++17
#add_definitions(-std=c++11)
set(CMAKE_CXX_STANDARD 11 PARENT_SCOPE)
set(CMAKE_CXX_STANDARD_REQUIRED ON PARENT_SCOPE)
set(CMAKE_CXX_EXTENSIONS OFF PARENT_SCOPE)
# use -std=c++14 instead of -std=gnu++14

# Print project compile flags
message(STATUS "Default Compile flags [current build: ${CMAKE_BUILD_TYPE}]")
message(STATUS "  CMAKE_C_FLAGS_DEBUG: ${CMAKE_C_FLAGS_DEBUG}")
message(STATUS "  CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(
    STATUS "  CMAKE_C_FLAGS_RELWITHDEBINFO: ${CMAKE_C_FLAGS_RELWITHDEBINFO}")
message(STATUS "  CMAKE_C_FLAGS_MINSIZEREL: ${CMAKE_C_FLAGS_MINSIZEREL}")

message(STATUS "  CMAKE_CXX_FLAGS_DEBUG: ${CMAKE_CXX_FLAGS_DEBUG}")
message(STATUS "  CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
message(
    STATUS "  CMAKE_CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}"
    )
message(STATUS "  CMAKE_CXX_FLAGS_MINSIZEREL: ${CMAKE_CXX_FLAGS_MINSIZEREL}")
message(STATUS "")

# Make a compiler flag for the current source folder path
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -D__SOURCE_FOLDER__='\"$(dir $(abspath $<))\"'")

define_property(TARGET
                PROPERTY LIBRARY_OPTIONS
                BRIEF_DOCS "List of options for a particular library"
                FULL_DOCS "List of options for a particular library")

define_property(
    TARGET
    PROPERTY CONFIGURABLE_LIBRARY_OPTIONS
    BRIEF_DOCS "List of configurable options for a particular library"
    FULL_DOCS "List of configurable options for a particular library")

function(ADD_LIBRARY_OPTION LIBRARY OPTION EXTRA_DEPENDENCIES)
    #if(${LIBRARY}_OPTIONS)
    #    set(${LIBRARY}_OPTIONS "${${LIBRARY}_OPTIONS} ${OPTION}" PARENT_SCOPE)
    #else()
    #    set(${LIBRARY}_OPTIONS "${OPTION}" PARENT_SCOPE)
    #endif()
    #set_property(GLOBAL APPEND PROPERTY LIBRARY_OPTIONS "${LIBRARY}_USE_${OPTION}")

    string(TOUPPER ${LIBRARY} LIBRARY_U)

    target_compile_definitions(
        ${LIBRARY} INTERFACE
        $<$<IN_LIST:${LIBRARY_U}_USE_${OPTION},$<TARGET_PROPERTY:LIBRARY_OPTIONS>>:"${LIBRARY_U}_USE_${OPTION}">
        )

    target_link_libraries(
        ${LIBRARY}
        INTERFACE
            $<$<IN_LIST:${LIBRARY_U}_USE_${OPTION},$<TARGET_PROPERTY:LIBRARY_OPTIONS>>:${EXTRA_DEPENDENCIES}>
        )
endfunction()

function(ENABLE_OPTION TARGET LIBRARY OPTION)
    # get_property(options GLOBAL PROPERTY LIBRARY_OPTIONS)
    # list(FILTER options INCLUDE REGEX "^${LIBRARY}_USE_(.*)$")

    # if(NOT options)
    #     message(FATAL_ERROR "No options found for library ${LIBRARY}")
    # endif()

    # if(NOT (${LIBRARY}_USE_${OPTION} IN_LIST options))
    #     message(FATAL_ERROR "Option ${OPTION} not found for library ${LIBRARY}")
    # endif()
    # if(NOT TARGET ${TARGET})
    #     message(FATAL_ERROR "Target ${TARGET} not found")
    # endif()
    string(TOUPPER ${LIBRARY} LIBRARY_U)

    message(STATUS "Adding compile definition: ${LIBRARY_U}_USE_${OPTION}")
    #target_compile_definitions(${TARGET} PUBLIC ${LIBRARY_U}_USE_${OPTION})
    set_property(TARGET ${TARGET}
                 APPEND
                 PROPERTY LIBRARY_OPTIONS "${LIBRARY_U}_USE_${OPTION}")

    # For debugging
    if(NOT TARGET print_options_${TARGET})
        add_custom_target(
            print_options_${TARGET}
            COMMAND ${CMAKE_COMMAND} -E echo
                    '$<TARGET_PROPERTY:${TARGET},LIBRARY_OPTIONS>')
    endif()
endfunction()

function(ADD_CONFIGURABLE_LIBRARY_OPTION LIBRARY OPTION)
    string(TOUPPER ${LIBRARY} LIBRARY_U)
    target_compile_definitions(
        ${LIBRARY} INTERFACE
        $<FILTER:$<TARGET_PROPERTY:CONFIGURABLE_LIBRARY_OPTIONS>,INCLUDE,^${LIBRARY_U}_${OPTION}=.*$>
        )
endfunction()

function(SET_CONFIGURABLE_OPTION TARGET LIBRARY OPTION VALUE)
    message(STATUS "Adding compile definition: ${LIBRARY}_${OPTION}=${VALUE}")
    set_property(TARGET ${TARGET}
                 APPEND
                 PROPERTY CONFIGURABLE_LIBRARY_OPTIONS
                          "${LIBRARY}_${OPTION}=${VALUE}")

    # For debugging
    if(NOT TARGET print_configurable_options_${TARGET})
        add_custom_target(
            print_configurable_options_${TARGET}
            COMMAND ${CMAKE_COMMAND} -E echo
                    '$<TARGET_PROPERTY:${TARGET},CONFIGURABLE_LIBRARY_OPTIONS>')
    endif()
endfunction()

# Macro to find all CMakeLists directories
macro(CMAKE_DIRECTORIES return_list)
    file(GLOB_RECURSE new_list CMakeLists.txt)
    set(dir_list "")
    foreach(file_path ${new_list})
        get_filename_component(dir_path ${file_path} PATH)
        set(dir_list ${dir_list} ${dir_path})
    endforeach()
    list(REMOVE_DUPLICATES dir_list)
    list(REMOVE_ITEM dir_list ${CMAKE_CURRENT_SOURCE_DIR})
    set(${return_list} ${dir_list})
endmacro()

cmake_directories(SUBDIRS)

message(STATUS "Loaded STM32-libraries:")
foreach(subdir ${SUBDIRS})
    set(LIBRARY_NAME "")
    if(${VERBOSE})
        message(STATUS "   " ${subdir})
    endif()
    add_subdirectory(${subdir})
    if(TARGET ${LIBRARY_NAME})
        target_include_directories(
            ${LIBRARY_NAME}
            INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    endif()
endforeach()
message(STATUS "")
