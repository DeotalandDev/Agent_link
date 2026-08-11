// ESP32-P4 Waveshare
#include "board.h"
#include "config.h"
#include "co5300_panel.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "ESP32P4Waveshare"

class ESP32P4Waveshare : public Board {
public:
    ESP32P4Waveshare() {
        InitDisplay();
    }

    const char* Name() const override { return "ESP32P4_WAVESHARE"; }

    uint32_t Capabilities() const override { return AGENT_CAP_SCREEN; }  // screen only for now

    // Agent asks the device to show text: at this reference stage a blue fill stands in for "received" (font rendering TODO).
    void ShowText(const char* utf8) override {
        ESP_LOGI(TAG, "ShowText \"%s\" (reference stage: blue fill for now; TODO font rendering)", utf8 ? utf8 : "");
        if (panel_.Ready()) panel_.FillSolid(0x001F);  // blue
    }

private:
    void InitDisplay() {
        Co5300Config c = {};
        c.rst_gpio    = DISPLAY_RST_PIN;      // reset pin (active low)
        c.pwr_en_gpio = DISPLAY_PWR_EN_PIN;   // VCI_EN panel power enable (active high)
        c.width       = DISPLAY_WIDTH;
        c.height      = DISPLAY_HEIGHT;
        if (panel_.Init(c) != ESP_OK) { ESP_LOGE(TAG, "display init failed"); return; }
        // Boot self-test: red -> green -> blue -> white, 600ms each, ending on white (easy to eyeball that the panel is lit).
        const uint16_t seq[] = { 0xF800, 0x07E0, 0x001F, 0xFFFF };
        for (uint16_t col : seq) { panel_.FillSolid(col); vTaskDelay(pdMS_TO_TICKS(600)); }
        ESP_LOGI(TAG, "display lit + self-test done (should end on white)");
    }

    Co5300Panel panel_;
};

DECLARE_BOARD(ESP32P4Waveshare);
