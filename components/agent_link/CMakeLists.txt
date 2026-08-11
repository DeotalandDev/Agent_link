idf_component_register(
    SRCS
        "src/agent_link.cpp"
        "src/transport_ble.cpp"
        "src/transport_wifi.cpp"
        "src/protocol.cpp"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
    REQUIRES esp_common
    PRIV_REQUIRES log bt nvs_flash freertos
)
