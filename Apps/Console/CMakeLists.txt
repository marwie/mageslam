# Copyright (c) Microsoft Corporation.
# Licensed under the MIT License.

add_executable(Console "console.cpp")
target_link_libraries(Console
    PRIVATE MAGESLAM)
