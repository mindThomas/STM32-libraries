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
