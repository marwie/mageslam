# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

if(OpenCV_DIR)
    find_package(OpenCV REQUIRED)
endif()

add_executable(Console "console.cpp")
target_link_libraries(Console
    PRIVATE opencv_world
    PRIVATE MAGESLAM)
