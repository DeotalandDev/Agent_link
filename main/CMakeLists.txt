set(SOURCES "app_main.cpp")
set(INCLUDE_DIRS "." "${CMAKE_CURRENT_SOURCE_DIR}/../boards/common")

file(GLOB COMMON_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/../boards/common/*.cc
    ${CMAKE_CURRENT_SOURCE_DIR}/../boards/common/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../boards/common/*.c)
list(APPEND SOURCES ${COMMON_SOURCES})

if(CONFIG_BOARD_TYPE_ROROLEE_S3)
    set(BOARD_DIR "rorolee-s3")
elseif(CONFIG_BOARD_TYPE_ESP32P4_WAVESHARE)
    set(BOARD_DIR "esp32p4Waveshare")
elseif(CONFIG_BOARD_TYPE_ESP32S3_TEM_MONITOR)
    set(BOARD_DIR "tem_monitor")
elseif(CONFIG_BOARD_TYPE_ESP32S3_ES8311_VOICE)
    set(BOARD_DIR "es8311-voice")
elseif(CONFIG_BOARD_TYPE_ESP32S3_ES8311_ASR)
    set(BOARD_DIR "es8311-asr")
elseif(CONFIG_BOARD_TYPE_ESP32S3_GC2145_CAMERA)
    set(BOARD_DIR "gc2145-camera")
elseif(CONFIG_BOARD_TYPE_ESP32S3_ROROLEE_BASIC)
    set(BOARD_DIR "rorolee-basic")
# Add a new board:Add the line elseif(CONFIG_BOARD_TYPE_XXX) set(BOARD_DIR "your-board")
endif()

if(BOARD_DIR)
    file(GLOB BOARD_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/../boards/${BOARD_DIR}/*.cc
        ${CMAKE_CURRENT_SOURCE_DIR}/../boards/${BOARD_DIR}/*.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/../boards/${BOARD_DIR}/*.c
    )
    list(APPEND SOURCES ${BOARD_SOURCES})
elseif(NOT CMAKE_BUILD_EARLY_EXPANSION)
    message(FATAL_ERROR "If no development board is selected, run idf.py, then go to menuconfig → Agent Link Device → Board Type")
endif()

set(EXTRA_REQUIRES "")
if(CONFIG_BOARD_TYPE_ESP32P4_WAVESHARE)
    list(APPEND EXTRA_REQUIRES esp_lcd_co5300)
endif()
if(CONFIG_BOARD_TYPE_ESP32S3_GC2145_CAMERA)
    list(APPEND EXTRA_REQUIRES esp32_camera esp_driver_rmt)
endif()

idf_component_register(
    SRCS ${SOURCES}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES agent_link esp_driver_gpio esp_driver_spi esp_lcd esp_lcd_sh8501
             esp_codec_dev esp_driver_i2c esp_driver_i2s ${EXTRA_REQUIRES})

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-missing-field-initializers)
