# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

set(LIBRARY_NAME "minimal")

set(SOURCES
    "minimal.cpp")

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME} PUBLIC "Include")

target_link_libraries(${LIBRARY_NAME}
    PRIVATE MAGESLAM)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
