cmake_minimum_required(VERSION 3.16)
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

# Board selection: idf.py menuconfig -> Agent Link Device -> Board Type (see boards/README.md).
# Target-wide sdkconfig defaults (PSRAM on, flash size, ...) live in sdkconfig.defaults /
# sdkconfig.defaults.<target> - native ESP-IDF, applied automatically by `idf.py set-target`.
set(_sdkconfig "${CMAKE_CURRENT_LIST_DIR}/sdkconfig")
if(EXISTS "${_sdkconfig}")
    file(STRINGS "${_sdkconfig}" _sdkconfig_lines)

    set(_selected_board "")
    foreach(_line IN LISTS _sdkconfig_lines)
        if(_line MATCHES "^CONFIG_(BOARD_TYPE_[A-Z0-9_]+)=y$")
            set(_selected_board "${CMAKE_MATCH_1}")
        endif()
    endforeach()

    # Explicit per-board wants for every managed choice member, both ways - so switching boards
    # back and forth via menuconfig always converges to the right value, not just the first hop.
    set(_s3_boards
        BOARD_TYPE_ROROLEE_S3 BOARD_TYPE_ESP32S3_ROROLEE_BASIC BOARD_TYPE_ESP32S3_TEM_MONITOR
        BOARD_TYPE_ESP32S3_ES8311_VOICE BOARD_TYPE_ESP32S3_ES8311_ASR BOARD_TYPE_ESP32S3_GC2145_CAMERA)
    set(_ext_crys_boards BOARD_TYPE_ROROLEE_S3 BOARD_TYPE_ESP32S3_ROROLEE_BASIC BOARD_TYPE_ESP32S3_TEM_MONITOR)

    set(_board_forced_options "")
    if("${_selected_board}" IN_LIST _s3_boards)
        list(APPEND _board_forced_options "CONFIG_SPIRAM_MODE_OCT=y" "CONFIG_SPIRAM_SPEED_80M=y")
        if("${_selected_board}" IN_LIST _ext_crys_boards)
            list(APPEND _board_forced_options "CONFIG_RTC_CLK_SRC_EXT_CRYS=y")
        else()
            list(APPEND _board_forced_options "CONFIG_RTC_CLK_SRC_INT_RC=y")
        endif()
    endif()

    set(_to_append "")
    foreach(_opt IN LISTS _board_forced_options)
        if(NOT "${_opt}" IN_LIST _sdkconfig_lines)
            list(APPEND _to_append "${_opt}")
        endif()
    endforeach()

    if(_to_append)
        list(JOIN _to_append "\n" _to_append_text)
        file(APPEND "${_sdkconfig}" "\n# board-forced options, patched by CMakeLists.txt (Kconfig select can't force choice members)\n${_to_append_text}\n")
        message(STATUS "agent_link: patched sdkconfig for ${_selected_board}: ${_to_append}")
    endif()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(agent_link)
