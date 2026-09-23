#include "application.h"
#include "button.h"
#include "codecs/no_audio_codec.h"
#include "config.h"
#include "smartmedivend_display.h"
#include "smartmedivend_pcm.h"
#include "wifi_board.h"

#include <driver/gpio.h>
#include <driver/spi_common.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_timer.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <limits>
#include <vector>

namespace {
// Keep the supplied GPIO map unchanged; none of the vending/MUX GPIOs are driven.
constexpr int kActivePins[] = {
    smv::pins::TFT_CS, smv::pins::TFT_RST, smv::pins::TFT_DC,
    smv::pins::TFT_MOSI, smv::pins::TFT_SCLK, smv::pins::TFT_BACKLIGHT,
    smv::pins::BUTTON, smv::pins::MIC_SD, smv::pins::MIC_WS,
    smv::pins::MIC_SCK, smv::pins::SPK_LRC, smv::pins::SPK_BCLK,
    smv::pins::SPK_DIN, smv::pins::MUX_S0, smv::pins::MUX_S1,
    smv::pins::MUX_S2, smv::pins::MUX_S3, smv::pins::MUX_SIG,
};
constexpr bool PinsAreSafe() {
    for (unsigned i = 0; i < sizeof(kActivePins) / sizeof(kActivePins[0]); ++i) {
        if (kActivePins[i] < 0 || kActivePins[i] > 48 ||
            (kActivePins[i] >= 26 && kActivePins[i] <= 37) ||
            kActivePins[i] == 19 || kActivePins[i] == 20 ||
            kActivePins[i] == 43 || kActivePins[i] == 44)
            return false;
        for (unsigned j = i + 1; j < sizeof(kActivePins) / sizeof(kActivePins[0]); ++j)
            if (kActivePins[i] == kActivePins[j])
                return false;
    }
    return true;
}
static_assert(PinsAreSafe(), "SmartMediVend has duplicate or reserved ESP32-S3 GPIOs");
constexpr char kTag[] = "SmartMediVend";
}  // namespace

// Two independent hardware I2S controllers: RX microphone at 16 kHz,
// TX speaker at 24 kHz. Retain the upstream codec's known PCM conversion,
// while reusing the RX scratch buffer instead of allocating every 10 ms.
class SmartMediVendAudioCodec final : public NoAudioCodecSimplex {
public:
    using NoAudioCodecSimplex::NoAudioCodecSimplex;

protected:
    int Read(int16_t* dest, int samples) override {
        if (!dest || samples <= 0 || !rx_handle_)
            return 0;
        const size_t wanted = static_cast<size_t>(samples);
        if (scratch_.size() < wanted)
            scratch_.resize(wanted);

        size_t bytes_read = 0;
        const esp_err_t result = i2s_channel_read(rx_handle_, scratch_.data(),
                                                    wanted * sizeof(int32_t),
                                                    &bytes_read, 200);
        if (result != ESP_OK || bytes_read == 0) {
            ++read_errors_;
            ReportLevelsIfDue();
            return 0;
        }
        const size_t received = std::min(bytes_read / sizeof(int32_t), wanted);
        uint32_t sum = 0;
        uint32_t peak = 0;
        for (size_t i = 0; i < received; ++i) {
            const int16_t pcm = smv::MicToPcm16(scratch_[i]);
            dest[i] = pcm;
            const uint32_t level = pcm == std::numeric_limits<int16_t>::min() ? 32768u :
                                   static_cast<uint32_t>(pcm < 0 ? -pcm : pcm);
            sum += level;
            peak = std::max(peak, level);
        }
        if (received != 0) {
            mean_level_ = sum / static_cast<uint32_t>(received);
            peak_level_ = peak;
            total_samples_ += received;
        }
        ReportLevelsIfDue();
        return static_cast<int>(received);
    }

private:
    void ReportLevelsIfDue() {
        const int64_t now = esp_timer_get_time();
        if (last_report_us_ == 0) {
            last_report_us_ = now;
            return;
        }
        if (now - last_report_us_ < 5000000)
            return;
        // Only signal levels / transport errors. Never log raw microphone samples.
        ESP_LOGI("SMV-MIC", "pcm_mean=%lu pcm_peak=%lu read_errors=%lu samples=%lu",
                 static_cast<unsigned long>(mean_level_),
                 static_cast<unsigned long>(peak_level_),
                 static_cast<unsigned long>(read_errors_),
                 static_cast<unsigned long>(total_samples_));
        read_errors_ = total_samples_ = 0;
        last_report_us_ = now;
    }

    std::vector<int32_t> scratch_;
    int64_t last_report_us_ = 0;
    uint32_t read_errors_ = 0;
    uint32_t total_samples_ = 0;
    uint32_t mean_level_ = 0;
    uint32_t peak_level_ = 0;
};

// Voice-only board: the supplied archive omits all medical/relay/stock modules.
// Do not invent dispensing behavior or energize actuator GPIOs to work around
// missing dependencies. The on-screen vending status remains LOCKED.
class SmartMediVendBoard final : public WifiBoard {
public:
    SmartMediVendBoard()
        : talk_button_(BOOT_BUTTON_GPIO, SMARTMEDIVEND_BUTTON_ACTIVE_HIGH, 2000, 35) {
        InitializeSpiAndPanel();
        InitializeButton();
        GetBacklight()->RestoreBrightness();
    }

    AudioCodec* GetAudioCodec() override {
        static SmartMediVendAudioCodec codec(
            AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK,
            AUDIO_I2S_SPK_GPIO_DOUT, SMARTMEDIVEND_SPEAKER_SLOT,
            AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS,
            AUDIO_I2S_MIC_GPIO_DIN, SMARTMEDIVEND_MIC_SLOT);
        return &codec;
    }

    Display* GetDisplay() override { return display_; }

    Backlight* GetBacklight() override {
        static PwmBacklight backlight(DISPLAY_BACKLIGHT_PIN, DISPLAY_BACKLIGHT_OUTPUT_INVERT);
        return &backlight;
    }

private:
    Button talk_button_;
    std::atomic<bool> long_pressed_{false};
    SmartMediVendDisplay* display_ = nullptr;

    void InitializeButton() {
        talk_button_.OnPressDown([this]() { long_pressed_.store(false); });
        talk_button_.OnLongPress([this]() {
            long_pressed_.store(true);
            Application::GetInstance().Schedule([this]() { EnterWifiConfigMode(); });
        });
        talk_button_.OnClick([this]() {
            if (long_pressed_.load())
                return;
            auto& app = Application::GetInstance();
            // Same startup behavior as Xiaozhi's reference WiFi board.
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        talk_button_.OnDoubleClick([this]() {
            if (long_pressed_.load())
                return;
            Application::GetInstance().Schedule([this]() {
                if (display_)
                    display_->ToggleStatusPage();
            });
        });
    }

    void InitializeSpiAndPanel() {
        spi_bus_config_t bus = {};
        bus.mosi_io_num = DISPLAY_MOSI_PIN;
        bus.miso_io_num = GPIO_NUM_NC;
        bus.sclk_io_num = DISPLAY_CLK_PIN;
        bus.quadwp_io_num = GPIO_NUM_NC;
        bus.quadhd_io_num = GPIO_NUM_NC;
        bus.max_transfer_sz = DISPLAY_WIDTH * 20 * sizeof(uint16_t) + 32;
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus, SPI_DMA_CH_AUTO));

        esp_lcd_panel_io_handle_t io = nullptr;
        esp_lcd_panel_handle_t panel = nullptr;
        esp_lcd_panel_io_spi_config_t io_cfg = {};
        io_cfg.cs_gpio_num = DISPLAY_CS_PIN;
        io_cfg.dc_gpio_num = DISPLAY_DC_PIN;
        io_cfg.spi_mode = DISPLAY_SPI_MODE;
        io_cfg.pclk_hz = 20 * 1000 * 1000;
        io_cfg.trans_queue_depth = 10;
        io_cfg.lcd_cmd_bits = 8;
        io_cfg.lcd_param_bits = 8;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_cfg, &io));

        esp_lcd_panel_dev_config_t cfg = {};
        cfg.reset_gpio_num = DISPLAY_RST_PIN;
        cfg.rgb_ele_order = DISPLAY_RGB_ORDER;
        cfg.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &cfg, &panel));
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel, DISPLAY_INVERT_COLOR));
        ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel, DISPLAY_SWAP_XY));
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y));
        display_ = new SmartMediVendDisplay(io, panel, DISPLAY_WIDTH, DISPLAY_HEIGHT,
                                            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
                                            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
        ESP_LOGI(kTag, "ST7789 portrait %dx%d; voice-only; dispensing disabled",
                 DISPLAY_WIDTH, DISPLAY_HEIGHT);
    }
};

DECLARE_BOARD(SmartMediVendBoard);
