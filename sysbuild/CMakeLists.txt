# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: Apache-2.0

# Custom sysbuild entry point that ensures our custom board root is
# discovered BEFORE Zephyr's boards.cmake runs.  This avoids the need
# to pass -DBOARD_ROOT=... on every build command.

# CMAKE_CURRENT_LIST_DIR points to <app>/sysbuild/, so parent is the app root.
get_filename_component(APP_ROOT "${CMAKE_CURRENT_LIST_DIR}" DIRECTORY)

if(NOT BOARD_ROOT)
    set(BOARD_ROOT ${APP_ROOT} CACHE PATH
        "Custom board root (auto-set by sysbuild/CMakeLists.txt)")
else()
    list(APPEND BOARD_ROOT ${APP_ROOT})
endif()

find_package(Sysbuild REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(sysbuild LANGUAGES)
