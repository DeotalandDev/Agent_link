#pragma once
// SHT30 temperature/humidity sensor (Sensirion, I2C). Single-shot measurement, clock stretching off, high repeatability.
#include "driver/i2c_master.h"
#include "esp_err.h"

class Sht30 {
public:
    // Attach to an existing I2C bus. addr: see config.h (0x44/0x45).
    esp_err_t Init(i2c_master_bus_handle_t bus, uint8_t addr);
    // Trigger one single-shot measurement and read back temperature (C) + relative humidity (%RH). Either pointer may be NULL.
    esp_err_t Read(float* temp_c, float* humidity_rh);
    bool Ready() const { return dev_ != nullptr; }

private:
    i2c_master_dev_handle_t dev_ = nullptr;
};
