// SHT30 driver implementation (see sht30.h).
// Sequence: write a 2-byte measurement command -> wait for the measurement -> read 6 bytes {T_msb, T_lsb, T_crc, RH_msb, RH_lsb, RH_crc}.
#include "sht30.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "SHT30";
constexpr int kI2cTimeoutMs = 1000;

// Sensirion CRC-8: polynomial 0x31, init 0xFF, MSB first.
uint8_t Crc8(const uint8_t* data, int len) {
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b)
            crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x31)
                               : static_cast<uint8_t>(crc << 1);
    }
    return crc;
}
}  // namespace

esp_err_t Sht30::Init(i2c_master_bus_handle_t bus, uint8_t addr) {
    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr;
    dc.scl_speed_hz    = 400000;
    esp_err_t r = i2c_master_bus_add_device(bus, &dc, &dev_);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "add_device @0x%02x failed: %s", addr, esp_err_to_name(r));
        dev_ = nullptr;
        return r;
    }
    // Soft reset 0x30A2; wait ~2ms to settle.
    const uint8_t soft_reset[2] = { 0x30, 0xA2 };
    i2c_master_transmit(dev_, soft_reset, sizeof soft_reset, kI2cTimeoutMs);
    vTaskDelay(pdMS_TO_TICKS(2));
    ESP_LOGI(TAG, "ready @0x%02x", addr);
    return ESP_OK;
}

esp_err_t Sht30::Read(float* temp_c, float* humidity_rh) {
    if (!dev_) return ESP_ERR_INVALID_STATE;

    // Single-shot, clock stretching disabled, high repeatability: command 0x2400.
    const uint8_t cmd[2] = { 0x24, 0x00 };
    esp_err_t r = i2c_master_transmit(dev_, cmd, sizeof cmd, kI2cTimeoutMs);
    if (r != ESP_OK) return r;

    vTaskDelay(pdMS_TO_TICKS(20));   // a high-repeatability measurement takes up to ~15ms

    uint8_t d[6] = {};
    r = i2c_master_receive(dev_, d, sizeof d, kI2cTimeoutMs);
    if (r != ESP_OK) return r;

    if (Crc8(&d[0], 2) != d[2] || Crc8(&d[3], 2) != d[5]) {
        ESP_LOGW(TAG, "CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    const uint16_t raw_t = static_cast<uint16_t>((d[0] << 8) | d[1]);
    const uint16_t raw_h = static_cast<uint16_t>((d[3] << 8) | d[4]);
    if (temp_c)      *temp_c      = -45.0f + 175.0f * (raw_t / 65535.0f);
    if (humidity_rh) *humidity_rh = 100.0f * (raw_h / 65535.0f);
    return ESP_OK;
}
