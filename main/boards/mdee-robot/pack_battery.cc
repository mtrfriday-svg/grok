#include "pack_battery.h"
#include "config.h"
#include "mdee_util.h"
#include "power_seq.h"
#include "sound_fx.h"
#include "light_rig.h"
#include "status_screen.h"

#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "PackBat"

static int PercentFromCell(float cell) {
    const float v[7] = {4.20f, 4.00f, 3.85f, 3.75f, 3.65f, 3.50f, 3.30f};
    const int p[7] = {100, 80, 55, 35, 20, 8, 0};
    if (cell >= v[0]) return 100;
    if (cell <= v[6]) return 0;
    for (int i = 0; i < 6; i++) {
        if (cell <= v[i] && cell >= v[i + 1]) {
            float span = v[i] - v[i + 1];
            if (span < 0.001f) return p[i];
            float t = (cell - v[i + 1]) / span;
            return (int)(p[i + 1] + t * (float)(p[i] - p[i + 1]) + 0.5f);
        }
    }
    return 0;
}

PackBattery& PackBattery::GetInstance() {
    static PackBattery instance;
    return instance;
}

float PackBattery::Volts() const { return mv_ / 1000.0f; }

float PackBattery::VoltsOrAssumed() const {
    if (!connected_ || mv_ < (int)(PACK_NO_PACK_VOLTS * 1000.0f)) return MOTOR_ASSUME_PACK_V;
    return mv_ / 1000.0f;
}

void PackBattery::Init() {
    if (inited_) return;
    inited_ = true;
    xTaskCreate(&PackBattery::Task, "pack_bat", 4096, this, 4, nullptr);
}

void PackBattery::Task(void* arg) {
    static_cast<PackBattery*>(arg)->Step();
    vTaskDelete(nullptr);
}

void PackBattery::Step() {
    adc_oneshot_unit_handle_t adc = nullptr;
    adc_cali_handle_t cali = nullptr;
    adc_oneshot_unit_init_cfg_t uc = {};
    uc.unit_id = PACK_ADC_UNIT;
    if (adc_oneshot_new_unit(&uc, &adc) != ESP_OK) {
        ESP_LOGW(TAG, "ADC init failed");
        adc = nullptr;
    } else {
        adc_oneshot_chan_cfg_t cc = {};
        cc.atten = ADC_ATTEN_DB_12;
        cc.bitwidth = ADC_BITWIDTH_DEFAULT;
        adc_oneshot_config_channel(adc, PACK_ADC_CHANNEL, &cc);
        adc_cali_curve_fitting_config_t cal = {};
        cal.unit_id = PACK_ADC_UNIT;
        cal.chan = PACK_ADC_CHANNEL;
        cal.atten = ADC_ATTEN_DB_12;
        cal.bitwidth = ADC_BITWIDTH_DEFAULT;
        if (adc_cali_create_scheme_curve_fitting(&cal, &cali) != ESP_OK) cali = nullptr;
    }

    float filtered = 0.0f;
    bool have = false;
    while (true) {
        if (adc != nullptr) {
            long sum = 0;
            int n = 0;
            for (int i = 0; i < 16; i++) {
                int raw = 0;
                if (adc_oneshot_read(adc, PACK_ADC_CHANNEL, &raw) == ESP_OK) {
                    sum += raw;
                    n++;
                }
            }
            if (n > 0) {
                int mv = 0;
                int raw = (int)(sum / n);
                if (cali == nullptr || adc_cali_raw_to_voltage(cali, raw, &mv) != ESP_OK) {
                    mv = raw * 3100 / 4095;
                }
                float pin_v = mv / 1000.0f;
                float pack = pin_v * (PACK_R_UPPER + PACK_R_LOWER) / PACK_R_LOWER * BATTERY_CAL_FACTOR;
                if (!have) {
                    filtered = pack;
                    have = true;
                } else {
                    filtered = filtered * 0.8f + pack * 0.2f;
                }
                mv_ = (int)(filtered * 1000.0f);
            }
        }

        int64_t now = MdeeNowMs();
        bool connected = have && filtered >= PACK_NO_PACK_VOLTS;
        connected_ = connected;
        if (!connected) {
            percent_ = 0;
            cutoff_ = false;
            low_ = false;
            low_v_since_ = 0;
            critical_since_ = 0;
            low_pct_since_ = 0;
            low_announced_ = false;
            LightRig::GetInstance().SetLowBattery(false);
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        int pct = PercentFromCell(filtered * 0.5f);
        percent_ = MdeeClamp(pct, 0, 100);

#if MOTOR_LOW_V_CUTOFF
        if (filtered < MOTOR_CUTOFF_VOLTS) {
            if (low_v_since_ == 0) low_v_since_ = now;
            cutoff_ = (now - low_v_since_) >= MOTOR_CUTOFF_MS;
        } else if (filtered > MOTOR_CUTOFF_VOLTS + 0.20f) {
            low_v_since_ = 0;
            cutoff_ = false;
        }
#else
        cutoff_ = false;
        low_v_since_ = 0;
#endif

        if (percent_ <= PACK_LOW_PCT) {
            if (low_pct_since_ == 0) low_pct_since_ = now;
            if ((now - low_pct_since_) >= PACK_LOW_MS) {
                low_ = true;
                LightRig::GetInstance().SetLowBattery(true);
                StatusScreen::GetInstance().SetAlert("LOW BATTERY");
                if (!low_announced_) {
                    low_announced_ = true;
                    SoundFx::GetInstance().Play(Sfx::kLowBat);
                }
            }
        } else if (percent_ >= PACK_LOW_PCT + 5) {
            low_pct_since_ = 0;
            low_ = false;
            low_announced_ = false;
            LightRig::GetInstance().SetLowBattery(false);
            StatusScreen::GetInstance().ClearAlertIf("LOW BATTERY");
        }

        if (filtered < PACK_CRITICAL_VOLTS) {
            if (critical_since_ == 0) critical_since_ = now;
            StatusScreen::GetInstance().SetAlert("CHARGE NOW");
            if (!off_sent_ && (now - critical_since_) >= PACK_CRITICAL_MS) {
                off_sent_ = true;
                ESP_LOGW(TAG, "Critical pack %.2f V, soft power-off", filtered);
                PowerSeq::GetInstance().PowerOff();
            }
        } else if (filtered > PACK_CRITICAL_VOLTS + 0.15f) {
            critical_since_ = 0;
            StatusScreen::GetInstance().ClearAlertIf("CHARGE NOW");
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
