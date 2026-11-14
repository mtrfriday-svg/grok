#include "light_rig.h"
#include "config.h"
#include "mdee_util.h"
#include "assets/lang_config.h"

#include <led_strip.h>
#include <driver/ledc.h>
#include <esp_log.h>
#include <esp_random.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <math.h>

#define TAG "Lights"

static int Scale(int v, int pct) {
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return (v * pct) / 100;
}

LightRig& LightRig::GetInstance() {
    static LightRig instance;
    return instance;
}

void LightRig::RgbFromHsv(int h, uint8_t& r, uint8_t& g, uint8_t& b) {
    h %= 360;
    if (h < 0) h += 360;
    int region = h / 60;
    int rem = h % 60;
    int q = (255 * (60 - rem)) / 60;
    int t = (255 * rem) / 60;
    switch (region) {
        case 0: r = 255; g = (uint8_t)t; b = 0; break;
        case 1: r = (uint8_t)q; g = 255; b = 0; break;
        case 2: r = 0; g = 255; b = (uint8_t)t; break;
        case 3: r = 0; g = (uint8_t)q; b = 255; break;
        case 4: r = (uint8_t)t; g = 0; b = 255; break;
        default: r = 255; g = 0; b = (uint8_t)q; break;
    }
}

bool LightRig::ParseEffect(const char* name, BarEffect& out) const {
    if (name == nullptr) return false;
    if (strcmp(name, "off") == 0) out = BarEffect::kOff;
    else if (strcmp(name, "solid") == 0 || strcmp(name, "on") == 0) out = BarEffect::kSolid;
    else if (strcmp(name, "breathe") == 0) out = BarEffect::kBreathe;
    else if (strcmp(name, "rainbow") == 0) out = BarEffect::kRainbow;
    else if (strcmp(name, "heartbeat") == 0 || strcmp(name, "heart") == 0) out = BarEffect::kHeartbeat;
    else if (strcmp(name, "flash") == 0 || strcmp(name, "blink") == 0) out = BarEffect::kFlash;
    else if (strcmp(name, "chase") == 0) out = BarEffect::kChase;
    else if (strcmp(name, "sparkle") == 0) out = BarEffect::kSparkle;
    else if (strcmp(name, "auto") == 0) out = BarEffect::kBreathe;
    else return false;
    return true;
}

bool LightRig::ParseBlue(const char* name, BlueMode& out) const {
    if (name == nullptr) return false;
    if (strcmp(name, "off") == 0) out = BlueMode::kOff;
    else if (strcmp(name, "on") == 0) out = BlueMode::kOn;
    else if (strcmp(name, "breathe") == 0) out = BlueMode::kBreathe;
    else if (strcmp(name, "blink") == 0) out = BlueMode::kBlink;
    else if (strcmp(name, "pulse") == 0) out = BlueMode::kPulse;
    else if (strcmp(name, "auto") == 0) out = BlueMode::kAuto;
    else return false;
    return true;
}

void LightRig::Init() {
    if (inited_) return;
    inited_ = true;
    led_strip_config_t sc = {};
    sc.strip_gpio_num = NEOPIXEL_PIN;
    sc.max_leds = NEOPIXEL_COUNT;
    sc.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    sc.led_model = LED_MODEL_WS2812;
    led_strip_rmt_config_t rc = {};
    rc.resolution_hz = 10 * 1000 * 1000;
    rc.mem_block_symbols = 64;
    led_strip_handle_t handle = nullptr;
    if (led_strip_new_rmt_device(&sc, &rc, &handle) == ESP_OK) {
        strip_ = handle;
        led_strip_clear(handle);
    } else {
        ESP_LOGW(TAG, "NeoPixel init failed");
        strip_ = nullptr;
    }

    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_10_BIT;
    t.timer_num = LEDC_TIMER_2;
    t.freq_hz = 5000;
    t.clk_cfg = LEDC_AUTO_CLK;
    t.deconfigure = false;
    ledc_channel_config_t c = {};
    c.gpio_num = BLUE_LED_PIN;
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = LEDC_CHANNEL_5;
    c.intr_type = LEDC_INTR_DISABLE;
    c.timer_sel = LEDC_TIMER_2;
    c.duty = 0;
    c.hpoint = 0;
    c.flags.output_invert = 0;
    if (ledc_timer_config(&t) != ESP_OK || ledc_channel_config(&c) != ESP_OK) {
        ESP_LOGW(TAG, "Blue LED PWM failed");
    }
    xTaskCreate(&LightRig::Task, "lights", 4096, this, 3, nullptr);
}

void LightRig::SetBars(const char* which, BarEffect effect, uint8_t r, uint8_t g, uint8_t b) {
    bool left = true;
    bool right = true;
    if (which != nullptr && strcmp(which, "left") == 0) right = false;
    else if (which != nullptr && strcmp(which, "right") == 0) left = false;
    bars_manual_ = true;
    perf_rainbow_ = false;
    if (left) {
        effect_[0] = effect;
        cr_[0] = r; cg_[0] = g; cb_[0] = b;
    }
    if (right) {
        effect_[1] = effect;
        cr_[1] = r; cg_[1] = g; cb_[1] = b;
    }
}

void LightRig::SetBarsAuto(bool on) {
    bars_manual_ = !on;
    if (on) perf_rainbow_ = false;
}

void LightRig::SetBlue(BlueMode mode) {
    blue_ = mode;
    blue_manual_ = mode != BlueMode::kAuto;
}

void LightRig::SetBrightness(const char* target, int percent) {
    percent = MdeeClamp(percent, 0, 100);
    bool bars = false;
    bool blue = false;
    bool screens = false;
    if (target == nullptr || strcmp(target, "all") == 0) {
        bars = blue = screens = true;
    } else if (strcmp(target, "bars") == 0) bars = true;
    else if (strcmp(target, "blue") == 0) blue = true;
    else if (strcmp(target, "screens") == 0) screens = true;
    if (bars) bar_bright_ = percent;
    if (blue) blue_bright_ = percent;
    if (screens) {
        // Backlight is applied by the caller when target includes screens,
        // because this class does not own the PwmBacklight object.
    }
    (void)screens;
}

void LightRig::SetThinking(bool on) { thinking_ = on; }
void LightRig::SetLowBattery(bool on) { low_battery_ = on; }
void LightRig::SetPerformanceRainbow(bool on) { perf_rainbow_ = on; }

void LightRig::ApplyStatus(const char* status) {
    if (status == nullptr) return;
    int kind = 0;
    if (strcmp(status, Lang::Strings::LISTENING) == 0) kind = 1;
    else if (strcmp(status, Lang::Strings::SPEAKING) == 0) kind = 2;
    else if (strcmp(status, Lang::Strings::CONNECTING) == 0) kind = 3;
    auto_kind_ = kind;
    if (!bars_manual_ && !perf_rainbow_) {
        effect_[0] = effect_[1] = BarEffect::kBreathe;
        if (kind == 1) { cr_[0] = cr_[1] = 0; cg_[0] = cg_[1] = 40; cb_[0] = cb_[1] = 255; }
        else if (kind == 2) { cr_[0] = cr_[1] = 0; cg_[0] = cg_[1] = 220; cb_[0] = cb_[1] = 60; }
        else if (kind == 3) { cr_[0] = cr_[1] = 160; cg_[0] = cg_[1] = 40; cb_[0] = cb_[1] = 255; }
        else { cr_[0] = cr_[1] = 20; cg_[0] = cg_[1] = 216; cb_[0] = cb_[1] = 255; }
    }
}

void LightRig::AllOff() {
    bars_manual_ = true;
    blue_manual_ = true;
    perf_rainbow_ = false;
    effect_[0] = effect_[1] = BarEffect::kOff;
    blue_ = BlueMode::kOff;
    bar_bright_ = 0;
    blue_bright_ = 0;
    led_strip_handle_t strip = static_cast<led_strip_handle_t>(strip_);
    if (strip != nullptr) {
        led_strip_clear(strip);
        led_strip_refresh(strip);
    }
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
}

void LightRig::Task(void* arg) {
    auto* self = static_cast<LightRig*>(arg);
    while (true) {
        self->Tick();
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void LightRig::PaintBar(int bar, int64_t now) {
    led_strip_handle_t strip = static_cast<led_strip_handle_t>(strip_);
    if (strip == nullptr) return;
    int i0 = bar == 0 ? 0 : 2;
    BarEffect effect = effect_[bar];
    uint8_t r = cr_[bar], g = cg_[bar], b = cb_[bar];
    if (perf_rainbow_ && !bars_manual_) effect = BarEffect::kRainbow;
    if (low_battery_ && !bars_manual_ && !perf_rainbow_) {
        effect = BarEffect::kBreathe;
        r = 255; g = 0; b = 0;
    }
    float env = 1.0f;
    switch (effect) {
        case BarEffect::kOff:
            env = 0.0f;
            break;
        case BarEffect::kSolid:
            env = 1.0f;
            break;
        case BarEffect::kBreathe: {
            float w = low_battery_ ? 1400.0f : 700.0f;
            env = 0.12f + 0.88f * (0.5f + 0.5f * sinf((float)now / w));
            break;
        }
        case BarEffect::kHeartbeat: {
            int ph = (int)(now % 1000);
            if (ph < 120) env = ph / 120.0f;
            else if (ph < 240) env = 1.0f - (ph - 120) / 120.0f;
            else if (ph < 360) env = 0.65f * (ph - 240) / 120.0f;
            else if (ph < 500) env = 0.65f * (1.0f - (ph - 360) / 140.0f);
            else env = 0.04f;
            break;
        }
        case BarEffect::kFlash:
            env = ((now / 160) & 1) ? 1.0f : 0.0f;
            break;
        case BarEffect::kChase:
        case BarEffect::kSparkle:
        case BarEffect::kRainbow:
            env = 1.0f;
            break;
        default:
            env = 0.0f;
            break;
    }
    int bright = Scale(100, bar_bright_);
    for (int i = 0; i < 2; i++) {
        uint8_t rr = r, gg = g, bb = b;
        float pix = env;
        if (effect == BarEffect::kRainbow) {
            int hue = (int)((now / 8 + (i0 + i) * 40) % 360);
            RgbFromHsv(hue, rr, gg, bb);
        } else if (effect == BarEffect::kChase) {
            int on = (int)((now / 140) & 1);
            pix = (i == on) ? 1.0f : 0.05f;
        } else if (effect == BarEffect::kSparkle) {
            uint32_t rnd = esp_random();
            pix = ((rnd >> (i + bar * 3)) & 3u) == 0u ? 1.0f : 0.0f;
        }
        int k = (int)(pix * (float)bright);
        if (k < 0) k = 0;
        if (k > 100) k = 100;
        led_strip_set_pixel(strip, i0 + i, (uint8_t)Scale(rr, k), (uint8_t)Scale(gg, k), (uint8_t)Scale(bb, k));
    }
}

void LightRig::PaintBlue(int64_t now) {
    float env = 0.0f;
    BlueMode mode = blue_;
    if (!blue_manual_) {
        if (low_battery_) mode = BlueMode::kBlink;
        else if (thinking_) mode = BlueMode::kBlink;
        else if (auto_kind_ == 1) mode = BlueMode::kPulse;
        else mode = BlueMode::kOff;
    }
    switch (mode) {
        case BlueMode::kOff:
        case BlueMode::kAuto:
            env = 0.0f;
            break;
        case BlueMode::kOn:
            env = 1.0f;
            break;
        case BlueMode::kBreathe:
            env = 0.15f + 0.85f * (0.5f + 0.5f * sinf((float)now / 600.0f));
            break;
        case BlueMode::kBlink: {
            int period = low_battery_ && !blue_manual_ ? 700 : 280;
            env = ((now / period) & 1) ? 1.0f : 0.0f;
            break;
        }
        case BlueMode::kPulse: {
            int ph = (int)(now % 900);
            if (ph < 180) env = ph / 180.0f;
            else if (ph < 420) env = 1.0f - (ph - 180) / 240.0f;
            else env = 0.0f;
            break;
        }
        default:
            env = 0.0f;
            break;
    }
    int duty = (int)(env * (float)Scale(1023, blue_bright_));
    if (duty < 0) duty = 0;
    if (duty > 1023) duty = 1023;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, (uint32_t)duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
}

void LightRig::Tick() {
    int64_t now = MdeeNowMs();
    PaintBar(0, now);
    PaintBar(1, now);
    led_strip_handle_t strip = static_cast<led_strip_handle_t>(strip_);
    if (strip != nullptr) led_strip_refresh(strip);
    PaintBlue(now);
}
