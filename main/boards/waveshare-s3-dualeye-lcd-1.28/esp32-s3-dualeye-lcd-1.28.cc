// esp32-s3-dualeye-lcd-1.28.cc (FINAL v9.3 - Cách A, Smooth GIF + No-Decoder)
// Dual round GC9A01 for Xiaozhi (LVGL v9.3, ESP-IDF v5.5)
// - Giữ kiến trúc Cách A của bạn: đổi icon -> mirror ngay; timer chỉ invalidate cho GIF mượt
// - Loại bỏ hoàn toàn lv_image_decoder_get_info (ngăn crash lodepng/memcmp)
// - Kích thước ảnh phải lấy từ chính object ảnh trái (không decode) -> tránh khựng khi chuyển GIF
// - swap_bytes=1 cho màn phải để đúng màu

#include "wifi_board.h"
#include "codecs/box_audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "led/led.h"
#include "wifi_station.h"
#include "power_save_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/i2c_master.h"
#include "driver/spi_common.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_gc9a01.h"
#include "esp_lvgl_port.h"

#include "lvgl.h"

#include <string>
#include <functional>

#define TAG "DualEyeLCD"

// ====== Tham số tối ưu hoá ======
#ifndef DISPLAY_SPI_SCLK_HZ
#define DISPLAY_SPI_SCLK_HZ (60 * 1000 * 1000)
#endif
#ifndef LINES_PER_CHUNK
#define LINES_PER_CHUNK 12
#endif
#ifndef TRANS_QUEUE_DEPTH
#define TRANS_QUEUE_DEPTH 12
#endif
#ifndef MIRROR_PERIOD_MS
#define MIRROR_PERIOD_MS 60 // timer chỉ để GIF animate
#endif

#define LEDC_MAX_DUTY 8191
#define BACKLIGHT_MAX 100

#ifndef GPIO_NUM_NC
#define GPIO_NUM_NC ((gpio_num_t) - 1)
#endif

#ifndef DISPLAY2_INVERT_COLOR
#define DISPLAY2_INVERT_COLOR 1
#endif

//==================================================
// Helpers LVGL 9.3 (không dùng decoder)
//==================================================
static inline const void *img_get_src(lv_obj_t *obj) { return lv_image_get_src(obj); }
static inline void img_set_src(lv_obj_t *obj, const void *src) { lv_image_set_src(obj, src); }

static lv_obj_t *find_first_image(lv_obj_t *parent)
{
    if (!parent)
        return nullptr;
    uint32_t n = lv_obj_get_child_cnt(parent);
    for (uint32_t i = 0; i < n; ++i)
    {
        lv_obj_t *c = lv_obj_get_child(parent, i);
        if (lv_obj_has_class(c, &lv_image_class))
            return c;
        if (auto *sub = find_first_image(c))
            return sub;
    }
    return nullptr;
}

//==================================================
// Primary kế thừa SpiLcdDisplay để có emoji + callback
//==================================================
class PrimaryDualEyeDisplay : public SpiLcdDisplay
{
public:
    using SpiLcdDisplay::SpiLcdDisplay;

    void SetEmotion(const char *emotion) override
    {
        last_emotion_ = (emotion && *emotion) ? emotion : "neutral";
        // Cập nhật UI trái như cũ
        SpiLcdDisplay::SetEmotion(last_emotion_.c_str());
        // Mirror ngay
        if (on_emotion_change_)
            on_emotion_change_(last_emotion_);
    }
    void SetEmotionCallback(std::function<void(const std::string &)> cb) { on_emotion_change_ = std::move(cb); }

private:
    std::string last_emotion_ = "neutral";
    std::function<void(const std::string &)> on_emotion_change_;
};

//==================================================
// Board
//==================================================
class WaveshareS3DualEyeLCD : public WifiBoard
{
public:
    WaveshareS3DualEyeLCD() : boot_button_(BOOT_BUTTON_GPIO)
    {
        ESP_LOGI(TAG, "Board init start (dual)");

        init_i2c();
        init_spi();

        // LEFT panel (primary)
        if (!create_panel(DISPLAY_CS_PIN, DISPLAY_RESET_PIN, /*out*/ io1_, /*out*/ panel1_))
            ESP_LOGE(TAG, "Create LEFT panel failed");
        display_left_ = new PrimaryDualEyeDisplay(
            io1_, panel1_,
            DISPLAY_WIDTH, DISPLAY_HEIGHT,
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y,
            DISPLAY_SWAP_XY);
        ESP_LOGI(TAG, "Panel1 ready.");

        // RIGHT panel (secondary)
        if (!create_panel(DISPLAY2_CS_PIN, DISPLAY2_RESET_PIN, /*out*/ io2_, /*out*/ panel2_))
            ESP_LOGE(TAG, "Create RIGHT panel failed");
        ESP_LOGI(TAG, "Panel2 ready.");

        add_secondary_display();

        // Đồng bộ NGAY khi đổi icon
        display_left_->SetEmotionCallback([this](const std::string &)
                                          {
            need_resync_ = true;
            Application::GetInstance().Schedule([this](){ this->instant_resync(); }); });
        need_resync_ = true;

        init_backlight();
        set_brightness(brightness_);
        init_buttons();

        // Timer chỉ invalidate để GIF animate
        const esp_timer_create_args_t targs = {
            .callback = &MirrorTimerCb,
            .arg = this,
            .dispatch_method = ESP_TIMER_TASK,
            .name = "dual_mirror",
            .skip_unhandled_events = true,
        };
        ESP_ERROR_CHECK(esp_timer_create(&targs, &mirror_timer_));
        ESP_ERROR_CHECK(esp_timer_start_periodic(mirror_timer_, MIRROR_PERIOD_MS * 1000));

        ESP_LOGI(TAG, "Board init done");
    }

    //==================== WifiBoard overrides ====================
    Led *GetLed() override
    {
        static NoLed led;
        return &led;
    }

    AudioCodec *GetAudioCodec() override
    {
        static BoxAudioCodec codec(
            i2c_bus_,
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_MCLK,
            AUDIO_I2S_GPIO_BCLK,
            AUDIO_I2S_GPIO_WS,
            AUDIO_I2S_GPIO_DOUT,
            AUDIO_I2S_GPIO_DIN,
            AUDIO_CODEC_PA_PIN,
            AUDIO_CODEC_ES8311_ADDR,
            AUDIO_CODEC_ES7210_ADDR,
            AUDIO_INPUT_REFERENCE);
        return &codec;
    }

    Display *GetDisplay() override { return display_left_; }
    Backlight *GetBacklight() override { return nullptr; }

    void SetPowerSaveMode(bool enabled) override
    {
        if (!enabled && power_save_timer_)
            power_save_timer_->WakeUp();
        WifiBoard::SetPowerSaveMode(enabled);
    }

private:
    //==================== HW: I2C / SPI ====================
    void init_i2c()
    {
        i2c_master_bus_config_t cfg = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = I2C_SDA_IO,
            .scl_io_num = I2C_SCL_IO,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .flags = {.enable_internal_pullup = true},
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &i2c_bus_));
    }

    void init_spi()
    {
        spi_bus_config_t buscfg = {
            .mosi_io_num = DISPLAY_MOSI_PIN,
            .miso_io_num = DISPLAY_MISO_PIN,
            .sclk_io_num = DISPLAY_SCLK_PIN,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = DISPLAY_WIDTH * LINES_PER_CHUNK * sizeof(uint16_t) + 8,
        };
        ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
        gpio_set_drive_capability((gpio_num_t)DISPLAY_SCLK_PIN, GPIO_DRIVE_CAP_3);
        gpio_set_drive_capability((gpio_num_t)DISPLAY_MOSI_PIN, GPIO_DRIVE_CAP_3);
        gpio_set_drive_capability((gpio_num_t)DISPLAY_DC_PIN, GPIO_DRIVE_CAP_3);
    }

    bool create_panel(int cs_pin, int rst_pin,
                      esp_lcd_panel_io_handle_t &out_io,
                      esp_lcd_panel_handle_t &out_panel)
    {
        esp_lcd_panel_io_spi_config_t io_cfg = {
            .cs_gpio_num = cs_pin,
            .dc_gpio_num = DISPLAY_DC_PIN,
            .spi_mode = DISPLAY_SPI_MODE,
            .pclk_hz = DISPLAY_SPI_SCLK_HZ,
            .trans_queue_depth = TRANS_QUEUE_DEPTH,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
        };
        if (esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &out_io) != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi failed");
            return false;
        }

        esp_lcd_panel_dev_config_t p_cfg = {
            .reset_gpio_num = rst_pin,
            .rgb_endian = LCD_RGB_ENDIAN_BGR,
            .bits_per_pixel = 16,
        };
        if (esp_lcd_new_panel_gc9a01(out_io, &p_cfg, &out_panel) != ESP_OK)
        {
            ESP_LOGE(TAG, "esp_lcd_new_panel_gc9a01 failed");
            return false;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_reset(out_panel));
        ESP_ERROR_CHECK(esp_lcd_panel_init(out_panel));

        bool inv = (cs_pin == DISPLAY_CS_PIN) ? DISPLAY_INVERT_COLOR : DISPLAY2_INVERT_COLOR;
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(out_panel, inv));

        bool mx = (cs_pin == DISPLAY_CS_PIN) ? DISPLAY_MIRROR_X : DISPLAY2_MIRROR_X;
        bool my = (cs_pin == DISPLAY_CS_PIN) ? DISPLAY_MIRROR_Y : DISPLAY2_MIRROR_Y;
        bool sxy = (cs_pin == DISPLAY_CS_PIN) ? DISPLAY_SWAP_XY : DISPLAY2_SWAP_XY;
        ESP_ERROR_CHECK(esp_lcd_panel_mirror(out_panel, mx, my));
        if (sxy)
            ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(out_panel, true));

        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(out_panel, true));
        return true;
    }

    //==================== Secondary display ====================
    void add_secondary_display()
    {
        lvgl_port_display_cfg_t cfg = {};
        cfg.io_handle = io2_;
        cfg.panel_handle = panel2_;
        cfg.control_handle = nullptr;
        cfg.buffer_size = (uint32_t)(DISPLAY_WIDTH * LINES_PER_CHUNK); // pixel count
        cfg.double_buffer = false;
        cfg.trans_size = 0;
        cfg.hres = DISPLAY_WIDTH;
        cfg.vres = DISPLAY_HEIGHT;
        cfg.monochrome = false;
        cfg.rotation.swap_xy = DISPLAY2_SWAP_XY;
        cfg.rotation.mirror_x = DISPLAY2_MIRROR_X;
        cfg.rotation.mirror_y = DISPLAY2_MIRROR_Y;
        cfg.color_format = LV_COLOR_FORMAT_RGB565;
        cfg.flags.buff_dma = 1;
        cfg.flags.buff_spiram = 0;
        cfg.flags.sw_rotate = 0;
        cfg.flags.swap_bytes = 1; // đúng màu màn phải
        cfg.flags.full_refresh = 0;
        cfg.flags.direct_mode = 0;

        disp2_ = lvgl_port_add_disp(&cfg);
        if (!disp2_)
        {
            ESP_LOGE(TAG, "lvgl_port_add_disp (right) failed");
            return;
        }

        Application::GetInstance().Schedule([this]()
                                            {
            if(!lvgl_port_lock(portMAX_DELAY)) return;

            this->scr2_ = lv_display_get_screen_active(this->disp2_);
            if(!this->scr2_) { lvgl_port_unlock(); return; }

            this->img2_ = lv_image_create(this->scr2_);
            lv_obj_set_size(this->img2_, 64, 64);
            lv_obj_center(this->img2_);

            // Load screen đúng display phải
            lv_display_t* old = lv_display_get_default();
            lv_display_set_default(this->disp2_);
            lv_screen_load(this->scr2_);
            lv_display_set_default(old);

            // Sync lần đầu (không chờ event)
            this->instant_resync();

            lvgl_port_unlock(); });
    }

    //==================== Mirror helpers (NO decoder) ====================
    bool fetch_left_image_src(lv_obj_t **out_img, const void **out_src)
    {
        *out_img = nullptr;
        *out_src = nullptr;

        lv_display_t *left_disp = lv_display_get_default();
        lv_obj_t *left_scr = lv_display_get_screen_active(left_disp);
        if (!left_scr)
            return false;

        if (!left_img_cached_ || !lv_obj_is_valid(left_img_cached_))
            left_img_cached_ = find_first_image(left_scr);
        if (!left_img_cached_)
            return false;

        const void *src = img_get_src(left_img_cached_);
        if (!src)
            return false;

        *out_img = left_img_cached_;
        *out_src = src;
        return true;
    }

    // LẤY SIZE TỪ ẢNH TRÁI (không gọi decoder) -> không khựng khi chuyển GIF/PNG
    void apply_src_to_secondary_opt(const void *src, lv_obj_t *left_img)
    {
        // Guard src cơ bản
        lv_image_src_t tp = lv_image_src_get_type(src);
        if (tp == LV_IMAGE_SRC_UNKNOWN)
        {
            lv_obj_add_flag(img2_, LV_OBJ_FLAG_HIDDEN);
            return;
        }
        if (tp == LV_IMAGE_SRC_VARIABLE)
        {
            const lv_image_dsc_t *d = static_cast<const lv_image_dsc_t *>(src);
            if (!d || !d->data || d->data_size == 0)
            {
                lv_obj_add_flag(img2_, LV_OBJ_FLAG_HIDDEN);
                return;
            }
        }

        // 1) set src (không reset nhiều lần)
        img_set_src(img2_, src);

        // 2) size theo object trái (đang hiển thị), không decode
        lv_coord_t w = left_img ? lv_obj_get_width(left_img) : DISPLAY_WIDTH;
        lv_coord_t h = left_img ? lv_obj_get_height(left_img) : DISPLAY_HEIGHT;
        if (w <= 0 || h <= 0)
        {
            w = DISPLAY_WIDTH;
            h = DISPLAY_HEIGHT;
        }

        if (w != last_w2_ || h != last_h2_)
        {
            lv_obj_set_size(img2_, w, h);
            lv_obj_center(img2_);
            last_w2_ = w;
            last_h2_ = h;
        }

        lv_obj_clear_flag(img2_, LV_OBJ_FLAG_HIDDEN);
        lv_obj_invalidate(img2_);
    }

    // Đồng bộ ngay lập tức (không chờ timer), NO decoder
    void instant_resync()
    {
        if (!disp2_ || !img2_)
            return;
        if (!lvgl_port_lock(pdMS_TO_TICKS(20)))
            return;

        lv_obj_t *left_img = nullptr;
        const void *src = nullptr;
        if (fetch_left_image_src(&left_img, &src))
        {
            if (src != last_left_src_)
            {
                apply_src_to_secondary_opt(src, left_img);
                last_left_src_ = src;
            }
            else
            {
                // src đã giống -> invalidate nhẹ cho đồng bộ khung
                lv_obj_clear_flag(img2_, LV_OBJ_FLAG_HIDDEN);
                lv_obj_invalidate(img2_);
            }
        }
        else
        {
            lv_obj_add_flag(img2_, LV_OBJ_FLAG_HIDDEN);
            last_left_src_ = nullptr;
        }

        lvgl_port_unlock();
        need_resync_ = false;
    }

    //==================== Timer animate (fallback nhẹ) ====================
    static void MirrorTimerCb(void *arg)
    {
        auto *self = static_cast<WaveshareS3DualEyeLCD *>(arg);
        Application::GetInstance().Schedule([self]()
                                            { self->mirror_step(); });
    }

    void mirror_step()
    {
        if (!disp2_ || !img2_)
            return;

        if (!lvgl_port_lock(pdMS_TO_TICKS(5)))
            return;

        // Nếu đang cần resync (vừa đổi icon), thử áp ngay (không decode)
        if (need_resync_)
        {
            lv_obj_t *left_img = nullptr;
            const void *src = nullptr;
            if (fetch_left_image_src(&left_img, &src))
            {
                if (src != last_left_src_)
                {
                    apply_src_to_secondary_opt(src, left_img);
                    last_left_src_ = src;
                }
            }
            need_resync_ = false;
        }

        // Luôn invalidate để GIF tiếp tục animate mượt (không thay src)
        lv_obj_invalidate(img2_);
        lvgl_port_unlock();
    }

    //==================== Backlight ====================
    void init_backlight()
    {
        ledc_timer_config_t tcfg = {
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .duty_resolution = LEDC_TIMER_13_BIT,
            .timer_num = LEDC_TIMER_0,
            .freq_hz = 5000,
            .clk_cfg = LEDC_AUTO_CLK,
        };
        ESP_ERROR_CHECK(ledc_timer_config(&tcfg));

        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC)
        {
            bl_ch1_ = {.gpio_num = DISPLAY_BACKLIGHT_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
            ESP_ERROR_CHECK(ledc_channel_config(&bl_ch1_));
        }
        if (DISPLAY2_BACKLIGHT_PIN != GPIO_NUM_NC)
        {
            bl_ch2_ = {.gpio_num = DISPLAY2_BACKLIGHT_PIN, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0};
            ESP_ERROR_CHECK(ledc_channel_config(&bl_ch2_));
        }
    }

    void set_brightness(uint8_t light)
    {
        if (light > BACKLIGHT_MAX)
            light = BACKLIGHT_MAX;
        uint32_t duty = (light * LEDC_MAX_DUTY) / BACKLIGHT_MAX;

        if (DISPLAY_BACKLIGHT_PIN != GPIO_NUM_NC)
        {
            ledc_set_duty(bl_ch1_.speed_mode, bl_ch1_.channel, duty);
            ledc_update_duty(bl_ch1_.speed_mode, bl_ch1_.channel);
        }
        if (DISPLAY2_BACKLIGHT_PIN != GPIO_NUM_NC)
        {
            ledc_set_duty(bl_ch2_.speed_mode, bl_ch2_.channel, duty);
            ledc_update_duty(bl_ch2_.speed_mode, bl_ch2_.channel);
        }
        brightness_ = light;
    }

    //==================== Buttons ====================
    void init_buttons()
    {
        boot_button_.OnClick([this]()
                             {
            auto& app = Application::GetInstance();
            if(app.GetDeviceState() == kDeviceStateStarting && !WifiStation::GetInstance().IsConnected()){
                this->ResetWifiConfiguration();
            }
            app.ToggleChatState(); });
    }

private:
    // HW
    i2c_master_bus_handle_t i2c_bus_ = nullptr;
    Button boot_button_;

    // Panels/IO
    esp_lcd_panel_io_handle_t io1_ = nullptr;
    esp_lcd_panel_handle_t panel1_ = nullptr;
    esp_lcd_panel_io_handle_t io2_ = nullptr;
    esp_lcd_panel_handle_t panel2_ = nullptr;

    // Primary & Secondary
    PrimaryDualEyeDisplay *display_left_ = nullptr;

    lv_display_t *disp2_ = nullptr;
    lv_obj_t *scr2_ = nullptr;
    lv_obj_t *img2_ = nullptr;

    // Cache & sync
    lv_obj_t *left_img_cached_ = nullptr;
    const void *last_left_src_ = nullptr;
    volatile bool need_resync_ = false;

    // Kích thước icon hiện tại (màn phải)
    lv_coord_t last_w2_ = -1, last_h2_ = -1;

    // Backlight
    ledc_channel_config_t bl_ch1_{};
    ledc_channel_config_t bl_ch2_{};
    uint8_t brightness_ = 85;

    // Timer animate
    esp_timer_handle_t mirror_timer_ = nullptr;

    PowerSaveTimer *power_save_timer_ = nullptr;
};

DECLARE_BOARD(WaveshareS3DualEyeLCD);
