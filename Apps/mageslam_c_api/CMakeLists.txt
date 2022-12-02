# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

set(LIBRARY_NAME "mageslam_c_api")

set(SOURCES
    "Source/test.cpp"
    "Include/mageslam_c_api.h"
    "Source/mageslam_c_api.cpp")

add_library(${LIBRARY_NAME} ${SOURCES})

target_include_directories(${LIBRARY_NAME} PUBLIC "Include")

target_link_libraries(${LIBRARY_NAME}
    PRIVATE MAGESLAM)

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} FILES ${SOURCES})
