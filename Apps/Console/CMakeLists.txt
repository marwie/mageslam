# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

find_package(OpenCV REQUIRED)

add_executable(Console "console.cpp")
target_link_libraries(Console
    PRIVATE opencv_world
    PRIVATE MAGESLAM)
