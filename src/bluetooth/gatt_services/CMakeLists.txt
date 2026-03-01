target_sources(app PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/device_info.c
           ${CMAKE_CURRENT_SOURCE_DIR}/battery_service.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/button_service.c
           ${CMAKE_CURRENT_SOURCE_DIR}/sensor_service.c
           ${CMAKE_CURRENT_SOURCE_DIR}/led_service.cpp
           ${CMAKE_CURRENT_SOURCE_DIR}/meow_ctrl_service.cpp
)

# Audio config service only needed with audio enabled
target_sources_ifdef(CONFIG_NRF5340_AUDIO app PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/audio_config_service.c
)
