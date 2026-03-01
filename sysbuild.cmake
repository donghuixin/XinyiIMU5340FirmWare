# sysbuild.cmake — Ensure BOARD_ROOT points to this repository so the
# custom openearable_v2 board definition under boards/teco/ is found
# by the sysbuild-level CMake without having to pass -DBOARD_ROOT on
# the command line.

if(NOT BOARD_ROOT)
    set(BOARD_ROOT ${APP_DIR} CACHE PATH "Custom board root (auto-set by sysbuild.cmake)")
endif()
