// TemMonitor — pressure / temperature / humidity monitoring board.
//   Hardware: SPA06 (pressure/temperature) + SHT30 (temperature/humidity) on one I2C bus,
//             plus an ST7789 240x240 SPI display.
//   agent_link: reports through the generic I/O interface (register_io declares endpoints,
//   push_reading reports periodically); Capabilities is AGENT_CAP_SENSOR. SPA06's temperature
//   is used only for its own pressure compensation; the reported temperature comes from SHT30.
#include "board.h"
#include "config.h"
#include "spa06.h"
#include "sht30.h"
#include "st7789_panel.h"

#include "agent_link.h"          // lifecycle + generic I/O: register_io / push_reading
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "TemMonitor"

class TemMonitorBoard : public Board {
public:
    TemMonitorBoard() {
        InitDisplay();      // ST7789 SPI panel
        InitI2c();
        InitSensors();
        RegisterIo();       // must run before agent_link_start() (the ctor runs before init/start, so this holds)
        StartSensorTask();  // read both sensors -> agent_link_push_reading
        StartDisplayTest(); // screen refresh (solid-fill) test
    }

    const char* Name() const override { return "TemMonitor"; }
    uint32_t Capabilities() const override { return AGENT_CAP_SENSOR; }  // sensing only for now (no text rendering yet)

private:
    // ── ST7789 display ──
    void InitDisplay() {
        St7789Config c = {};
        c.spi_host = DISPLAY_SPI_HOST;
        c.pin_sck  = DISPLAY_PIN_SCK;
        c.pin_mosi = DISPLAY_PIN_MOSI;
        c.pin_cs   = DISPLAY_PIN_CS;
        c.pin_dc   = DISPLAY_PIN_DC;
        c.pin_rst  = DISPLAY_PIN_RST;
        c.pin_bl   = DISPLAY_PIN_BL;
        c.width    = DISPLAY_WIDTH;
        c.height   = DISPLAY_HEIGHT;
        c.pclk_hz  = DISPLAY_SPI_CLK_HZ;
        // Color inversion / order / orientation use ST7789 defaults; flip invert_color, bgr,
        // mirror_*, swap_xy, or gap_* in the config if the picture looks wrong.
        display_ok_ = (display_.Init(c) == ESP_OK);
        if (!display_ok_) ESP_LOGW(TAG, "ST7789 init failed — screen test skipped");
    }

    // ── One I2C bus; each sensor is a separate device on it ──
    void InitI2c() {
        i2c_master_bus_config_t bus = {};
        bus.i2c_port          = I2C_PORT;
        bus.sda_io_num        = I2C_SDA;
        bus.scl_io_num        = I2C_SCL;
        bus.clk_source        = I2C_CLK_SRC_DEFAULT;
        bus.glitch_ignore_cnt = 7;
        bus.flags.enable_internal_pullup = true;
        if (i2c_new_master_bus(&bus, &bus_) != ESP_OK) {
            ESP_LOGE(TAG, "i2c bus init failed — sensors unavailable");
            bus_ = nullptr;
        }
    }

    void InitSensors() {
        if (!bus_) return;
        spa_ok_ = (spa_.Init(bus_, SPA06_ADDR) == ESP_OK);
        sht_ok_ = (sht_.Init(bus_, SHT30_ADDR) == ESP_OK);
        if (!spa_ok_) ESP_LOGW(TAG, "SPA06 not ready — skipping pressure reports");
        if (!sht_ok_) ESP_LOGW(TAG, "SHT30 not ready — skipping temp/humidity reports");
    }

    // Endpoint descriptors are long-lived -> static storage (register_io only stores the pointer).
    // The cloud turns each into an MCP resource
    void RegisterIo() {
        static const agent_link_io_desc_t pres = {
            .id = "pres0", .dir = AGENT_IO_IN, .kind = "pressure",
            .value = AGENT_VAL_F32, .unit = "hPa", .desc = "atmospheric pressure (SPA06)",
            .range_min = 300.0f, .range_max = 1100.0f, .rate_hz = 1, .args_schema = nullptr,
        };
        static const agent_link_io_desc_t temp = {
            .id = "temp0", .dir = AGENT_IO_IN, .kind = "temperature",
            .value = AGENT_VAL_F32, .unit = "C", .desc = "ambient temperature (SHT30)",
            .range_min = -40.0f, .range_max = 125.0f, .rate_hz = 1, .args_schema = nullptr,
        };
        static const agent_link_io_desc_t hum = {
            .id = "hum0", .dir = AGENT_IO_IN, .kind = "humidity",
            .value = AGENT_VAL_F32, .unit = "%RH", .desc = "relative humidity (SHT30)",
            .range_min = 0.0f, .range_max = 100.0f, .rate_hz = 1, .args_schema = nullptr,
        };
        if (spa_ok_) agent_link_register_io(&pres, nullptr, nullptr);  // sensor -> cb = NULL
        if (sht_ok_) {
            agent_link_register_io(&temp, nullptr, nullptr);
            agent_link_register_io(&hum,  nullptr, nullptr);
        }
    }

    void StartSensorTask() {
        if (!spa_ok_ && !sht_ok_) { ESP_LOGW(TAG, "no usable sensor — not starting the sampling task"); return; }
        xTaskCreate(&SensorTaskEntry, "sensor", 4096, this, 4, &task_);
    }
    static void SensorTaskEntry(void* arg) { static_cast<TemMonitorBoard*>(arg)->SensorLoop(); }

    // 1 Hz: read both sensors -> report typed values (f32) by id. push_reading is safely ignored when not connected.
    void SensorLoop() {
        while (true) {
            if (spa_ok_) {
                float pa;
                if (spa_.Read(&pa, nullptr) == ESP_OK) {
                    const float hpa = pa / 100.0f;                       // Pa -> hPa (matches the unit)
                    agent_link_push_reading("pres0", &hpa, sizeof hpa);
                    ESP_LOGI(TAG, "pressure = %.2f hPa", hpa);
                }
            }
            if (sht_ok_) {
                float c, rh;
                if (sht_.Read(&c, &rh) == ESP_OK) {
                    agent_link_push_reading("temp0", &c,  sizeof c);
                    agent_link_push_reading("hum0",  &rh, sizeof rh);
                    ESP_LOGI(TAG, "temp = %.2f C  humidity = %.2f %%RH", c, rh);
                }
            }
            vTaskDelay(pdMS_TO_TICKS(1000));   // 1 Hz, matches rate_hz
        }
    }

    // ── Screen refresh test: cycle solid colors so the panel visibly repaints ──
    void StartDisplayTest() {
        if (!display_ok_) return;
        xTaskCreate(&DisplayTestEntry, "lcd_test", 4096, this, 3, &disp_task_);
    }
    static void DisplayTestEntry(void* arg) { static_cast<TemMonitorBoard*>(arg)->DisplayTestLoop(); }

    void DisplayTestLoop() {
        struct ColorStep { uint16_t color; const char* name; };
        static const ColorStep seq[] = {
            { rgb565::kRed,   "red"   },
            { rgb565::kGreen, "green" },
            { rgb565::kBlue,  "blue"  },
            { rgb565::kWhite, "white" },
            { rgb565::kBlack, "black" },
        };
        for (uint32_t round = 0; ; ++round) {
            for (const ColorStep& s : seq) {
                if (display_.FillSolid(s.color) == ESP_OK) {
                    ESP_LOGI(TAG, "screen test: %s (round %u)", s.name, static_cast<unsigned>(round));
                } else {
                    ESP_LOGW(TAG, "screen test: fill %s failed", s.name);
                }
                vTaskDelay(pdMS_TO_TICKS(500));
            }
        }
    }

    i2c_master_bus_handle_t bus_ = nullptr;
    Spa06 spa_;
    Sht30 sht_;
    bool  spa_ok_ = false;
    bool  sht_ok_ = false;
    St7789Panel  display_;
    bool         display_ok_ = false;
    TaskHandle_t task_      = nullptr;
    TaskHandle_t disp_task_ = nullptr;
};

DECLARE_BOARD(TemMonitorBoard);
