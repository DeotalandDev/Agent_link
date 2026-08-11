#pragma once
// TI BQ27220 single-cell Li-ion fuel gauge (I2C coulomb counter). Reusable driver in boards/common.
//
// Read-only usage (SOC / charging / voltage)
#include <cstdint>

#include "driver/i2c_master.h"
#include "esp_err.h"

class Bq27220 {
public:
    // addr7: 7-bit address (0x55)
    esp_err_t Init(i2c_port_t port, uint8_t addr7 = 0x55, uint32_t freq_hz = 100000);
    bool  Ready() const { return ready_; }

    int   Soc() const;
    bool  IsCharging() const;
    float VoltageVolts() const;

private:
    esp_err_t ReadU16Le(uint8_t reg, uint16_t* out) const;

    i2c_master_dev_handle_t dev_ = nullptr;
    bool ready_ = false;
};
