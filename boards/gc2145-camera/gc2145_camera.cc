// GC2145 Camera:live camera preview on an ST7789 240x240 SPI LCD
//
// A minimal example board: bring up the GC2145 DVP sensor (via Espressif's esp32-camera component) at RGB565 240x240
// a task that continuously grabs frames and blits them to the ST7789
// Frames live in PSRAM; the LCD DMAs directly from them

#include "board.h"
#include "config.h"
#include "st7789_lcd.h"

#include "esp_camera.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "Gc2145Cam"

class Gc2145CameraBoard : public Board {
public:
    Gc2145CameraBoard() {
        if (lcd_.Init(MakeLcdConfig()) != ESP_OK) { ESP_LOGE(TAG, "LCD init failed"); return; }
        lcd_ok_ = true;
        if (InitCamera() != ESP_OK) { ESP_LOGE(TAG, "camera init failed"); lcd_.FillSolid(rgb565::kRed); return; }
        cam_ok_ = true;
        xTaskCreate(&Gc2145CameraBoard::PreviewTaskEntry, "cam_preview", 4096, this, 5, &preview_task_);
    }

    const char* Name() const override { return "GC2145_CAMERA"; }

    // Hardware present: a camera and a screen
    uint32_t Capabilities() const override { return AGENT_CAP_CAMERA | AGENT_CAP_SCREEN; }

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
            // Orientation — flip these if the preview is upside down / mirrored on your module.
            s->set_vflip(s, 0);
            s->set_hmirror(s, 0);
        }
        ESP_LOGI(TAG, "camera ready (RGB565 240x240)");
        return ESP_OK;
    }

    static void PreviewTaskEntry(void* arg) { static_cast<Gc2145CameraBoard*>(arg)->PreviewLoop(); }

    void PreviewLoop() {
        ESP_LOGI(TAG, "preview loop started");
        uint32_t frames = 0;
        while (true) {
            camera_fb_t* fb = esp_camera_fb_get();
            if (!fb) { ESP_LOGW(TAG, "fb_get failed"); vTaskDelay(pdMS_TO_TICKS(10)); continue; }
#if CAMERA_RGB565_BYTE_SWAP
            // Camera RGB565 is little-endian per pixel; ST7789 latches big-endian. Swap in place.
            uint16_t* p = reinterpret_cast<uint16_t*>(fb->buf);
            for (size_t i = 0, n = fb->len / 2; i < n; ++i) p[i] = __builtin_bswap16(p[i]);
#endif
            // DrawBitmap blocks until the DMA finishes, so returning the fb right after is safe.
            (void)lcd_.DrawBitmap(0, 0, static_cast<uint16_t>(fb->width), static_cast<uint16_t>(fb->height), fb->buf);
            esp_camera_fb_return(fb);
            if ((++frames % 100) == 0) ESP_LOGD(TAG, "preview: %u frames", (unsigned)frames);
        }
    }

    St7789Lcd    lcd_;
    bool         lcd_ok_       = false;
    bool         cam_ok_       = false;
    TaskHandle_t preview_task_ = nullptr;
};

DECLARE_BOARD(Gc2145CameraBoard);
