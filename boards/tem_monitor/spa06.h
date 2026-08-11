#pragma once
// SPA06 pressure/temperature sensor (Goertek, I2C). SPL06-compatible plus the extended
// c31/c40 calibration coefficients. Runs in continuous-measurement mode; raw readings are
// compensated to physical units using the on-chip calibration coefficients.
#include "driver/i2c_master.h"
#include "esp_err.h"
#include <cstdint>

class Spa06 {
public:
    // Attach to an existing I2C bus and configure continuous measurement. addr: see config.h (0x76/0x77).
    esp_err_t Init(i2c_master_bus_handle_t bus, uint8_t addr);
    // Read one compensated sample: pressure (Pa) + temperature (C). Either pointer may be NULL.
    esp_err_t Read(float* pressure_pa, float* temp_c);
    bool Ready() const { return dev_ != nullptr; }

private:
    esp_err_t ReadRegs(uint8_t reg, uint8_t* buf, size_t n);
    esp_err_t WriteReg(uint8_t reg, uint8_t val);
    esp_err_t ReadCoefficients();

    i2c_master_dev_handle_t dev_ = nullptr;

    // Calibration coefficients (sign-extended integer values).
    int32_t c0_ = 0, c1_ = 0;
    int32_t c00_ = 0, c10_ = 0;
    int32_t c01_ = 0, c11_ = 0, c20_ = 0, c21_ = 0, c30_ = 0, c31_ = 0, c40_ = 0;
    // Oversampling scale factors (kP = pressure, kT = temperature); depend on the chosen OSR.
    float kP_ = 1.0f, kT_ = 1.0f;
};
