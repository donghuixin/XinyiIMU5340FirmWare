#
# Copyright (c) 2022 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

add_subdirectory(bt_management)

# LE Audio source compilation - only when audio is enabled
if(CONFIG_NRF5340_AUDIO)
add_subdirectory(bt_rendering_and_capture)
add_subdirectory(bt_content_control)
add_subdirectory(bt_stream)
endif()

add_subdirectory(gatt_services)

# Include directories must always be available (headers are referenced by zbus_common.h)
zephyr_library_include_directories(
    bt_management
    bt_rendering_and_capture
    bt_content_control
    bt_stream
    gatt_services
)
