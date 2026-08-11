// BQ27220 fuel gauge implementation
#include "bq27220.h"

#include "esp_log.h"

namespace {
constexpr const char* TAG = "Bq27220";
// BQ27220 standard commands (uint16 little-endian)
constexpr uint8_t kRegVoltage = 0x08;  // Voltage(): mV
constexpr uint8_t kRegCurrent = 0x0C;  // Current(): signed mA (positive = charging into battery, negative = discharging)
constexpr uint8_t kRegSoc     = 0x2C;  // StateOfCharge(): %
// Charge-detect current threshold (mA): filters the +/- few-mA rest noise; normal charge is hundreds of mA, trickle tens of mA, unplugged goes negative.
constexpr int16_t kChargeThreshMa = 10;
}  // namespace

esp_err_t Bq27220::Init(i2c_port_t port, uint8_t addr7, uint32_t freq_hz) {
    if (ready_) return ESP_OK;

    i2c_master_bus_handle_t bus = nullptr;
    esp_err_t r = i2c_master_get_bus_handle(port, &bus);
    if (r != ESP_OK || !bus) {
        ESP_LOGE(TAG, "no I2C bus on port %d (InitCodec must create the bus first): %s",
                 static_cast<int>(port), esp_err_to_name(r));
        return (r != ESP_OK) ? r : ESP_FAIL;
    }

    i2c_device_config_t dc = {};
    dc.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dc.device_address  = addr7;
    dc.scl_speed_hz    = freq_hz;
    r = i2c_master_bus_add_device(bus, &dc, &dev_);
    if (r != ESP_OK) { ESP_LOGE(TAG, "add_device: %s", esp_err_to_name(r)); return r; }

    uint16_t mv = 0;
    if (ReadU16Le(kRegVoltage, &mv) != ESP_OK) {
        ESP_LOGW(TAG, "probe(Voltage) failed; check wiring/pull-ups/whether the part is populated");
        i2c_master_bus_rm_device(dev_);
        dev_ = nullptr;
        return ESP_FAIL;
    }
    ready_ = true;
    ESP_LOGI(TAG, "BQ27220 ready, voltage=%u mV", static_cast<unsigned>(mv));
    return ESP_OK;
}

esp_err_t Bq27220::ReadU16Le(uint8_t reg, uint16_t* out) const {
    if (!dev_ || !out) return ESP_ERR_INVALID_STATE;
    uint8_t b[2];
    esp_err_t r = i2c_master_transmit_receive(dev_, &reg, 1, b, 2, 80);
    if (r != ESP_OK) return r;
    *out = static_cast<uint16_t>(b[0] | (static_cast<uint16_t>(b[1]) << 8));
    return ESP_OK;
}

int Bq27220::Soc() const {
    if (!ready_) return -1;
    uint16_t w = 0;
    if (ReadU16Le(kRegSoc, &w) == ESP_OK) {
        if (w <= 100)  return static_cast<int>(w);
        if (w <= 1000) return static_cast<int>(w / 10);
    }
    ESP_LOGW(TAG, "SOC read failed/invalid");
    return -1;
}

bool Bq27220::IsCharging() const {
    if (!ready_) return false;
    uint16_t raw = 0;
    if (ReadU16Le(kRegCurrent, &raw) != ESP_OK) return false;  // read failed: conservatively treat as not charging
    const int16_t current_ma = static_cast<int16_t>(raw);      // signed mA
    return current_ma > kChargeThreshMa;
}

float Bq27220::VoltageVolts() const {
    if (!ready_) return -1.0f;
    uint16_t mv = 0;
    if (ReadU16Le(kRegVoltage, &mv) != ESP_OK) return -1.0f;
    return static_cast<float>(mv) / 1000.0f;
}
