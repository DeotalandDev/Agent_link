// SPA06 driver implementation (see spa06.h). Registers and compensation formulas follow the SPA06-003 datasheet.
#include "spa06.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

namespace {
constexpr const char* TAG = "SPA06";
constexpr int kI2cTimeoutMs = 1000;

// Register addresses
constexpr uint8_t REG_PSR_B2   = 0x00;  // pressure, 24-bit: 0x00..0x02
constexpr uint8_t REG_TMP_B2   = 0x03;  // temperature, 24-bit: 0x03..0x05
constexpr uint8_t REG_PRS_CFG  = 0x06;
constexpr uint8_t REG_TMP_CFG  = 0x07;
constexpr uint8_t REG_MEAS_CFG = 0x08;
constexpr uint8_t REG_CFG_REG  = 0x09;
constexpr uint8_t REG_RESET    = 0x0C;
constexpr uint8_t REG_ID       = 0x0D;
constexpr uint8_t REG_COEF     = 0x10;  // calibration coefficients: 0x10..0x24 (21 bytes)
constexpr uint8_t REG_COEF_SRC = 0x28;  // bit7 = temperature-coefficient source (internal/external)

// Config: pressure 8/s @ 16x oversampling; temperature 8/s @ 1x oversampling.
constexpr uint8_t PRS_CFG_VAL = (0x3 << 4) | 0x4;  // rate = 8Hz, OSR = 16x
constexpr uint8_t TMP_RATE     = (0x3 << 4);        // rate = 8Hz (OSR = 1x -> bits[2:0] = 0)
constexpr uint8_t MEAS_CTRL_CONT_PT = 0x7;          // continuous measurement: pressure + temperature

// When OSR > 8 the result is >20 bits, so the matching SHIFT bit must be set in CFG_REG. Pressure 16x -> P_SHIFT (bit2); temperature 1x -> none.
constexpr uint8_t CFG_REG_VAL = 0x04;

// Oversampling -> scale factor (kP/kT). Pressure 16x, temperature 1x.
constexpr float K_OSR_16X = 253952.0f;
constexpr float K_OSR_1X  = 524288.0f;

// Sign-extend a 'bits'-wide unsigned value (two's complement) to int32.
int32_t Sext(uint32_t v, int bits) {
    const uint32_t m = 1u << (bits - 1);
    return static_cast<int32_t>((v ^ m) - m);
}
}  // namespace

esp_err_t Spa06::ReadRegs(uint8_t reg, uint8_t* buf, size_t n) {
    return i2c_master_transmit_receive(dev_, &reg, 1, buf, n, kI2cTimeoutMs);
}

esp_err_t Spa06::WriteReg(uint8_t reg, uint8_t val) {
    const uint8_t b[2] = { reg, val };
    return i2c_master_transmit(dev_, b, sizeof b, kI2cTimeoutMs);
}

esp_err_t Spa06::Init(i2c_master_bus_handle_t bus, uint8_t addr) {
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

    // Soft reset (write 0x09 to the RESET register); wait for the restart and coefficient load.
    WriteReg(REG_RESET, 0x09);
    vTaskDelay(pdMS_TO_TICKS(50));

    uint8_t id = 0;
    if (ReadRegs(REG_ID, &id, 1) == ESP_OK)
        ESP_LOGI(TAG, "chip id = 0x%02x", id);   // SPL06 = 0x10 / SPA06 family; logged only, not enforced

    // Wait for SENSOR_RDY + COEF_RDY (MEAS_CFG bit6/bit7).
    for (int i = 0; i < 100; ++i) {
        uint8_t m = 0;
        if (ReadRegs(REG_MEAS_CFG, &m, 1) == ESP_OK && (m & 0xC0) == 0xC0) break;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    r = ReadCoefficients();
    if (r != ESP_OK) { ESP_LOGE(TAG, "read coefficients failed"); return r; }

    // The temperature-coefficient source bit (COEF_SRCE bit7) must match TMP_CFG's TMP_EXT (bit7), or the temperature reads wrong.
    uint8_t src = 0;
    ReadRegs(REG_COEF_SRC, &src, 1);
    const uint8_t tmp_ext = src & 0x80;

    WriteReg(REG_PRS_CFG, PRS_CFG_VAL);
    WriteReg(REG_TMP_CFG, static_cast<uint8_t>(tmp_ext | TMP_RATE));
    WriteReg(REG_CFG_REG, CFG_REG_VAL);
    WriteReg(REG_MEAS_CFG, MEAS_CTRL_CONT_PT);

    kP_ = K_OSR_16X;
    kT_ = K_OSR_1X;
    ESP_LOGI(TAG, "ready @0x%02x (continuous P+T)", addr);
    return ESP_OK;
}

esp_err_t Spa06::ReadCoefficients() {
    uint8_t c[21] = {};
    esp_err_t r = ReadRegs(REG_COEF, c, sizeof c);   // 0x10..0x24
    if (r != ESP_OK) return r;

    c0_  = Sext((c[0] << 4) | (c[1] >> 4), 12);
    c1_  = Sext(((c[1] & 0x0F) << 8) | c[2], 12);
    c00_ = Sext((c[3] << 12) | (c[4] << 4) | (c[5] >> 4), 20);
    c10_ = Sext(((c[5] & 0x0F) << 16) | (c[6] << 8) | c[7], 20);
    c01_ = Sext((c[8] << 8) | c[9], 16);
    c11_ = Sext((c[10] << 8) | c[11], 16);
    c20_ = Sext((c[12] << 8) | c[13], 16);
    c21_ = Sext((c[14] << 8) | c[15], 16);
    c30_ = Sext((c[16] << 8) | c[17], 16);
    c31_ = Sext((c[18] << 4) | (c[19] >> 4), 12);
    c40_ = Sext(((c[19] & 0x0F) << 8) | c[20], 12);
    return ESP_OK;
}

esp_err_t Spa06::Read(float* pressure_pa, float* temp_c) {
    if (!dev_) return ESP_ERR_INVALID_STATE;

    uint8_t d[6] = {};
    esp_err_t r = ReadRegs(REG_PSR_B2, d, sizeof d);   // 0x00..0x05 = pressure + temperature
    if (r != ESP_OK) return r;

    const int32_t raw_p = Sext((d[0] << 16) | (d[1] << 8) | d[2], 24);
    const int32_t raw_t = Sext((d[3] << 16) | (d[4] << 8) | d[5], 24);

    const float t_sc = static_cast<float>(raw_t) / kT_;
    const float p_sc = static_cast<float>(raw_p) / kP_;

    // Temperature compensation (C)
    if (temp_c) *temp_c = static_cast<float>(c0_) * 0.5f + static_cast<float>(c1_) * t_sc;

    // Pressure compensation (Pa): full SPA06 order, including c40 (4th-order pressure) and c31 (3rd-order pressure/temperature cross term).
    if (pressure_pa) {
        *pressure_pa =
            static_cast<float>(c00_)
            + p_sc * (static_cast<float>(c10_)
                + p_sc * (static_cast<float>(c20_)
                    + p_sc * (static_cast<float>(c30_) + p_sc * static_cast<float>(c40_))))
            + t_sc * static_cast<float>(c01_)
            + t_sc * p_sc * (static_cast<float>(c11_)
                + p_sc * (static_cast<float>(c21_) + p_sc * static_cast<float>(c31_)));
    }
    return ESP_OK;
}
