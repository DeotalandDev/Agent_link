// GC2145 Camera:live camera preview on an ST7789 240x240 SPI LCD
//
// A minimal example board: bring up the GC2145 DVP sensor (via Espressif's esp32-camera component) at RGB565 240x240
// a task that continuously grabs frames and blits them to the ST7789
// Frames live in PSRAM; the ST7789 driver blits them out in stripes via an internal DMA buffer

#include "board.h"
#include "config.h"
#include "st7789_lcd.h"
#include "ws2812_led.h"

#include "esp_camera.h"
#include "img_converters.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "agent_link.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <atomic>
#include <cstdlib>
#include <cstring>

#define TAG "Gc2145Cam"

class Gc2145CameraBoard : public Board {
public:
    Gc2145CameraBoard() {
        InitLed();                 // independent of the camera; also doubles as a bring-up/fault indicator
        if (lcd_.Init(MakeLcdConfig()) != ESP_OK) { ESP_LOGE(TAG, "LCD init failed"); return; }
        lcd_ok_ = true;
        if (InitCamera() != ESP_OK) {
            ESP_LOGE(TAG, "camera init failed"); lcd_.FillSolid(rgb565::kRed);
            (void)led_.SetColor(16, 0, 0);   // dim red = camera fault
            return;
        }
        cam_ok_ = true;
        (void)led_.SetColor(0, 16, 0);       // dim green = camera up
        InitButton();              // BOOT key -> snapshot
        RegisterCaptureEndpoint(); // App/LLM -> snapshot (MCP actuator "camera0")
        xTaskCreate(&Gc2145CameraBoard::PreviewTaskEntry, "cam_preview", 4096, this, 5, &preview_task_);
    }

    const char* Name() const override { return "GC2145_CAMERA"; }

    // Hardware present: a camera, a screen, and the onboard WS2812 RGB LED
    uint32_t Capabilities() const override { return AGENT_CAP_CAMERA | AGENT_CAP_SCREEN | AGENT_CAP_LED; }

    // Agent -> Device: the SDK's synthetic "led0" endpoint routes here (0x00RRGGBB) -> drive the WS2812.
    void SetLed(uint32_t rgb) override { (void)led_.SetRgb(rgb); }

private:
    static St7789LcdConfig MakeLcdConfig() {
        St7789LcdConfig c = {};
        c.spi_host     = DISPLAY_SPI_HOST;
        c.pin_sck      = DISPLAY_SCK_PIN;
        c.pin_mosi     = DISPLAY_MOSI_PIN;
        c.pin_cs       = DISPLAY_CS_PIN;
        c.pin_dc       = DISPLAY_DC_PIN;
        c.pin_rst      = DISPLAY_RST_PIN;
        c.pin_bl       = DISPLAY_BL_PIN;
        c.width        = DISPLAY_WIDTH;
        c.height       = DISPLAY_HEIGHT;
        c.pclk_hz      = DISPLAY_SPI_CLK_HZ;
        c.spi_mode     = DISPLAY_SPI_MODE;
        c.invert_color = true;   // ST7789 default
        return c;
    }

    esp_err_t InitCamera() {
        camera_config_t cc = {};
        cc.pin_pwdn     = CAM_PIN_PWDN;
        cc.pin_reset    = CAM_PIN_RESET;
        cc.pin_xclk     = CAM_PIN_XCLK;
        cc.pin_sccb_sda = CAM_PIN_SIOD;
        cc.pin_sccb_scl = CAM_PIN_SIOC;
        cc.pin_d7       = CAM_PIN_D7;
        cc.pin_d6       = CAM_PIN_D6;
        cc.pin_d5       = CAM_PIN_D5;
        cc.pin_d4       = CAM_PIN_D4;
        cc.pin_d3       = CAM_PIN_D3;
        cc.pin_d2       = CAM_PIN_D2;
        cc.pin_d1       = CAM_PIN_D1;
        cc.pin_d0       = CAM_PIN_D0;
        cc.pin_vsync    = CAM_PIN_VSYNC;
        cc.pin_href     = CAM_PIN_HREF;
        cc.pin_pclk     = CAM_PIN_PCLK;
        cc.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
        cc.ledc_timer   = LEDC_TIMER_0;      // esp32-camera drives XCLK via LEDC
        cc.ledc_channel = LEDC_CHANNEL_0;
        cc.pixel_format = PIXFORMAT_RGB565;  // draw straight to the LCD, no decode
        cc.frame_size   = FRAMESIZE_240X240; // 1:1 with the screen
        cc.fb_count     = 2;                 // double-buffer in PSRAM
        cc.fb_location  = CAMERA_FB_IN_PSRAM;
        cc.grab_mode    = CAMERA_GRAB_LATEST;// always show the freshest frame
        esp_err_t r = esp_camera_init(&cc);
        if (r != ESP_OK) { ESP_LOGE(TAG, "esp_camera_init: %s", esp_err_to_name(r)); return r; }

        sensor_t* s = esp_camera_sensor_get();
        if (s) {
            ESP_LOGI(TAG, "camera sensor PID=0x%04x (GC2145 expected)", s->id.PID);
            // Orientation
            s->set_vflip(s, 0);
            s->set_hmirror(s, 0);
            RestartCaptureEngine(s);
        }
        ESP_LOGI(TAG, "camera ready (RGB565 240x240)");
        return ESP_OK;
    }

    // Pulsing CISCTL_restart_n (reg 0xfe bit4, active-low) low->high kicks it into streaming.
    static void RestartCaptureEngine(sensor_t* s) {
        if (!s->set_reg) return;
        s->set_reg(s, 0xfe, 0xff, 0x00);   // CISCTL restart asserted, page 0
        vTaskDelay(pdMS_TO_TICKS(20));
        s->set_reg(s, 0xfe, 0xff, 0x10);   // CISCTL restart released, page 0
        vTaskDelay(pdMS_TO_TICKS(20));
        ESP_LOGI(TAG, "GC2145 capture engine restarted");
    }

    static void LockExposureWhiteBalance(sensor_t* s) {
        if (!s->set_reg) return;
        s->set_reg(s, 0xfe, 0xff, 0x00);   // select register page 0
        s->set_reg(s, 0xb6, 0x01, 0x00);   // AEC enable (reg 0xb6 bit0) -> 0: hold exposure at the converged value
        s->set_reg(s, 0x82, 0x02, 0x00);   // AWB_en (reg 0x82 bit1) -> 0: hold white balance
        ESP_LOGI(TAG, "AEC/AWB locked — exposure & white balance frozen for a steady preview");
    }

    void RegisterCaptureEndpoint() {
        static agent_link_io_desc_t desc = {};
        desc.id           = "camera0";
        desc.dir          = AGENT_IO_OUT;
        desc.kind         = "camera.capture";
        desc.value        = AGENT_VAL_BOOL;
        desc.desc         = "Capture a still image from the camera and upload it to the app";
        desc.display_name = "Camera Snapshot";
        agent_link_register_io(&desc, &Gc2145CameraBoard::OnCaptureCmd, this);
    }

    // 0x33 IoActuate for "camera0". Runs on an SDK thread ,just flag the preview loop to do the capture + encode + send.
    static void OnCaptureCmd(const char* /*id*/, const uint8_t* /*args*/, size_t /*len*/, void* ctx) {
        auto* self = static_cast<Gc2145CameraBoard*>(ctx);
        if (self) { self->capture_req_.store(true, std::memory_order_release); ESP_LOGI(TAG, "snapshot requested (App)"); }
    }

    // Bring up the onboard WS2812 RGB LED (RMT). Safe if it fails — SetLed() just no-ops then.
    void InitLed() {
        if (led_.Init(WS2812_LED_PIN) != ESP_OK) { ESP_LOGE(TAG, "WS2812 init failed"); return; }
        (void)led_.Off();
        ESP_LOGI(TAG, "WS2812 LED ready on GPIO%d (App endpoint: led0)", (int)WS2812_LED_PIN);
    }

    // Snapshot button, polled in the preview loop with edge detection. This button is ACTIVE-HIGH
    void InitButton() {
        gpio_config_t c = {};
        c.pin_bit_mask = 1ULL << CAPTURE_BUTTON_PIN;
        c.mode         = GPIO_MODE_INPUT;
        c.pull_up_en   = GPIO_PULLUP_DISABLE;
        c.pull_down_en = GPIO_PULLDOWN_ENABLE;   // active-high: idle low, pressed = high
        c.intr_type    = GPIO_INTR_DISABLE;      // polled, not interrupt-driven
        gpio_config(&c);
        ESP_LOGI(TAG, "snapshot button ready: press GPIO%d to capture", (int)CAPTURE_BUTTON_PIN);
    }

    // Encode the current RGB565 frame to JPEG and hand it to the SDK's image channel
    // Must run on the ORIGINAL frame bytes, before any LCD byte-swap. send_image returns fast (async worker).
    void SendSnapshot(camera_fb_t* fb) {
        uint8_t* jpg = nullptr; size_t jpg_len = 0;
        if (!frame2jpg(fb, 80 /*quality*/, &jpg, &jpg_len)) { ESP_LOGE(TAG, "snapshot: frame2jpg failed"); return; }
        esp_err_t r = agent_link_send_image(jpg, jpg_len, AGENT_IMG_JPEG,
                                            static_cast<uint16_t>(fb->width), static_cast<uint16_t>(fb->height));
        ESP_LOGI(TAG, "snapshot: %ux%u -> %uB jpeg, send=%s",
                 (unsigned)fb->width, (unsigned)fb->height, (unsigned)jpg_len, esp_err_to_name(r));
        free(jpg);
    }

    static void PreviewTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->PreviewLoop(); }

    void PreviewLoop() {
        ESP_LOGI(TAG, "preview loop started");
        uint32_t frames = 0;
        bool btn_pressed_prev = false;   // active-high button idles low
        bool ae_locked = false;          // AEC/AWB frozen once converged (stops the static-scene "breathing")
        while (true) {
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) { ESP_LOGW(TAG, "fb_get failed"); vTaskDelay(pdMS_TO_TICKS(10)); continue; }

#if CAMERA_MASK_TOP_ROWS > 0
            // Overpaint the sensor's noisy top rows with the first clean row below, 
            // so the flickering black/white band is hidden in both the preview and any snapshot.
            // Done before everything else.
            {
                uint8_t* b = static_cast<uint8_t*>(fb->buf);
                const size_t rb = static_cast<size_t>(fb->width) * 2u;
                for (int r = 0; r < CAMERA_MASK_TOP_ROWS; ++r)
                    memcpy(b + static_cast<size_t>(r) * rb, b + static_cast<size_t>(CAMERA_MASK_TOP_ROWS) * rb, rb);
            }
#endif

            // Snapshot trigger: button rising edge (active-high; polled once per frame = natural debounce) or App command.
            const bool btn_pressed = (gpio_get_level(CAPTURE_BUTTON_PIN) != 0);   // high = pressed
            if (!btn_pressed_prev && btn_pressed) { capture_req_.store(true, std::memory_order_release); ESP_LOGI(TAG, "snapshot requested (button)"); }
            btn_pressed_prev = btn_pressed;
            // Encode + send from the ORIGINAL RGB565, before the LCD byte-swap below would corrupt the JPEG colors.
            if (capture_req_.exchange(false, std::memory_order_acq_rel)) SendSnapshot(fb);

#if CAMERA_RGB565_BYTE_SWAP
            // Camera RGB565 is little-endian per pixel; ST7789 latches big-endian. Swap in place.
            uint16_t* p = reinterpret_cast<uint16_t*>(fb->buf);
            for (size_t i = 0, n = fb->len / 2; i < n; ++i) p[i] = __builtin_bswap16(p[i]);
#endif
            // DrawBitmap blocks until the DMA finishes, so returning the fb right after is safe.
            (void)lcd_.DrawBitmap(0, 0, static_cast<uint16_t>(fb->width), static_cast<uint16_t>(fb->height), fb->buf);
            esp_camera_fb_return(fb);

            // Once AEC/AWB have had ~50 frames (~2s) to settle, lock them so the static scene stops drifting.
            if (!ae_locked && ++frames >= 50) {
                sensor_t* s = esp_camera_sensor_get();
                if (s) { LockExposureWhiteBalance(s); ae_locked = true; }
            }
        }
    }

    St7789Lcd         lcd_;
    Ws2812Led         led_;
    bool              lcd_ok_       = false;
    bool              cam_ok_       = false;
    TaskHandle_t      preview_task_ = nullptr;
    std::atomic<bool> capture_req_{false};   // set by App command / BOOT button; consumed by the preview loop
};

DECLARE_BOARD(Gc2145CameraBoard);
