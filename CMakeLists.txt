# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

cmake_minimum_required(VERSION 3.12)
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

project(MAGESLAM)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(Dependencies)
add_subdirectory(Core)

add_executable(Console "console.cpp")
target_link_libraries(Console
    PRIVATE MAGESLAM)
