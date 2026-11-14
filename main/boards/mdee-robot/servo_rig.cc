#include "servo_rig.h"
#include "config.h"
#include "mdee_util.h"

#include <driver/ledc.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

#define TAG "Servos"

static int ChFor(int index) { return LEDC_CHANNEL_1 + index; }

ServoRig& ServoRig::GetInstance() {
    static ServoRig instance;
    return instance;
}

float ServoRig::DownPose(int index) const {
    if (index == SERVO_IDX_TURN) return 50.0f;
    return 0.0f;
}

bool ServoRig::Enabled(int index) const {
    if (index < 0 || index > 3) return false;
    return enabled_[index];
}

float ServoRig::Pose(int index) const {
    if (index < 0 || index > 3) return 0.0f;
    return base_[index];
}

void ServoRig::BindChannel(int index) {
    ledc_channel_config_t c = {};
    c.gpio_num = (gpio_num_t)pin_[index];
    c.speed_mode = LEDC_LOW_SPEED_MODE;
    c.channel = (ledc_channel_t)ChFor(index);
    c.intr_type = LEDC_INTR_DISABLE;
    c.timer_sel = LEDC_TIMER_1;
    c.duty = 0;
    c.hpoint = 0;
    c.flags.output_invert = 0;
    ledc_channel_config(&c);
}

int ServoRig::AngleFor(int index, float pose) const {
    float p = MdeeClampf(pose, 0.0f, 100.0f);
    float a = (float)at0_[index] + ((float)at100_[index] - (float)at0_[index]) * (p / 100.0f);
    int angle = (int)(a + (a >= 0.0f ? 0.5f : -0.5f));
    return MdeeClamp(angle, SERVO_ABS_MIN, SERVO_ABS_MAX);
}

void ServoRig::WriteAngle(int index, int angle) {
    if (!enabled_[index] || !attached_) return;
    angle = MdeeClamp(angle, SERVO_ABS_MIN, SERVO_ABS_MAX);
    int pulse_us = 500 + (angle * 1900) / 180;
    if (pulse_us < 500) pulse_us = 500;
    if (pulse_us > 2400) pulse_us = 2400;
    uint32_t duty = ((uint32_t)pulse_us * 16384u) / 20000u;
    if (duty > 16383u) duty = 16383u;
    ledc_channel_t ch = (ledc_channel_t)ChFor(index);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
}

void ServoRig::Init() {
    if (inited_) return;
    inited_ = true;
    const int en[4] = {SERVO_TURN_ENABLED, SERVO_TILT_ENABLED, SERVO_EAR_L_ENABLED, SERVO_EAR_R_ENABLED};
    const int pins[4] = {(int)SERVO_TURN_PIN, (int)SERVO_TILT_PIN, (int)SERVO_EAR_L_PIN, (int)SERVO_EAR_R_PIN};
    const int a0[4] = {SERVO_TURN_AT_0, SERVO_TILT_AT_0, SERVO_EAR_L_AT_0, SERVO_EAR_R_AT_0};
    const int a1[4] = {SERVO_TURN_AT_100, SERVO_TILT_AT_100, SERVO_EAR_L_AT_100, SERVO_EAR_R_AT_100};
    const int dps[4] = {SERVO_NECK_MAX_DPS, SERVO_NECK_MAX_DPS, SERVO_EAR_MAX_DPS, SERVO_EAR_MAX_DPS};
    bool any = false;
    for (int i = 0; i < 4; i++) {
        enabled_[i] = en[i] != 0;
        pin_[i] = pins[i];
        at0_[i] = a0[i];
        at100_[i] = a1[i];
        max_dps_[i] = dps[i] > 0 ? dps[i] : 60;
        float down = DownPose(i);
        base_[i] = from_[i] = target_[i] = down;
        if (enabled_[i]) any = true;
    }
    if (!any) {
        ESP_LOGW(TAG, "All servos disabled in config.h");
        return;
    }
    ledc_timer_config_t t = {};
    t.speed_mode = LEDC_LOW_SPEED_MODE;
    t.duty_resolution = LEDC_TIMER_14_BIT;
    t.timer_num = LEDC_TIMER_1;
    t.freq_hz = 50;
    t.clk_cfg = LEDC_AUTO_CLK;
    t.deconfigure = false;
    if (ledc_timer_config(&t) != ESP_OK) {
        ESP_LOGE(TAG, "Servo timer failed");
        for (int i = 0; i < 4; i++) enabled_[i] = false;
        return;
    }
    Attach(true);
    xTaskCreate(&ServoRig::Task, "servos", 4096, this, 5, nullptr);
    ESP_LOGI(TAG, "Servos attached at down pose");
}

void ServoRig::Attach(bool on) {
    attached_ = on;
    for (int i = 0; i < 4; i++) {
        if (!enabled_[i]) continue;
        if (!on) {
            ledc_stop(LEDC_LOW_SPEED_MODE, (ledc_channel_t)ChFor(i), 0);
            continue;
        }
        float down = DownPose(i);
        base_[i] = from_[i] = target_[i] = down;
        moving_[i] = false;
        wiggle_amp_[i] = 0.0f;
        wiggle_until_[i] = 0;
        BindChannel(i);
        WriteAngle(i, AngleFor(i, down));
    }
}

void ServoRig::MoveTo(int index, float pose, int duration_ms) {
    if (index < 0 || index > 3 || !enabled_[index]) return;
    pose = MdeeClampf(pose, 0.0f, 100.0f);
    if (duration_ms < 1) duration_ms = 1;
    int a0 = AngleFor(index, base_[index]);
    int a1 = AngleFor(index, pose);
    int deg = a1 - a0;
    if (deg < 0) deg = -deg;
    int min_ms = (max_dps_[index] > 0) ? (deg * 1000) / max_dps_[index] : duration_ms;
    if (min_ms < 1) min_ms = 1;
    if (duration_ms < min_ms) duration_ms = min_ms;
    from_[index] = base_[index];
    target_[index] = pose;
    move_start_[index] = MdeeNowMs();
    move_ms_[index] = duration_ms;
    moving_[index] = true;
}

void ServoRig::MoveEars(float left, float right, int duration_ms) {
    MoveTo(SERVO_IDX_EAR_L, left, duration_ms);
    MoveTo(SERVO_IDX_EAR_R, right, duration_ms);
}

void ServoRig::MoveNeck(float turn, float tilt, int duration_ms) {
    MoveTo(SERVO_IDX_TURN, turn, duration_ms);
    MoveTo(SERVO_IDX_TILT, tilt, duration_ms);
}

void ServoRig::Wiggle(int index, float amplitude, int period_ms, int duration_ms) {
    if (index < 0 || index > 3 || !enabled_[index]) return;
    if (period_ms < 40) period_ms = 40;
    if (duration_ms < 40) duration_ms = 40;
    if (amplitude < 0.0f) amplitude = -amplitude;
    if (amplitude > 40.0f) amplitude = 40.0f;
    wiggle_amp_[index] = amplitude;
    wiggle_period_[index] = period_ms;
    wiggle_start_[index] = MdeeNowMs();
    wiggle_until_[index] = wiggle_start_[index] + duration_ms;
}

void ServoRig::WiggleEars(float amplitude, int period_ms, int duration_ms) {
    Wiggle(SERVO_IDX_EAR_L, amplitude, period_ms, duration_ms);
    Wiggle(SERVO_IDX_EAR_R, amplitude, period_ms, duration_ms);
}

void ServoRig::Perk(int side, int duration_ms) {
    if (side == 1) MoveTo(SERVO_IDX_EAR_L, 100.0f, duration_ms);
    else if (side == 2) MoveTo(SERVO_IDX_EAR_R, 100.0f, duration_ms);
    else MoveEars(100.0f, 100.0f, duration_ms);
}

void ServoRig::Droop(int side, int duration_ms) {
    if (side == 1) MoveTo(SERVO_IDX_EAR_L, 0.0f, duration_ms);
    else if (side == 2) MoveTo(SERVO_IDX_EAR_R, 0.0f, duration_ms);
    else MoveEars(0.0f, 0.0f, duration_ms);
}

void ServoRig::Neutral(int side, int duration_ms) {
    if (side == 1) MoveTo(SERVO_IDX_EAR_L, 35.0f, duration_ms);
    else if (side == 2) MoveTo(SERVO_IDX_EAR_R, 35.0f, duration_ms);
    else MoveEars(35.0f, 35.0f, duration_ms);
}

void ServoRig::Flick(int side) {
    int period = 160;
    int dur = 320;
    if (side == 1) Wiggle(SERVO_IDX_EAR_L, 22.0f, period, dur);
    else if (side == 2) Wiggle(SERVO_IDX_EAR_R, 22.0f, period, dur);
    else WiggleEars(22.0f, period, dur);
}

void ServoRig::Task(void* arg) {
    auto* self = static_cast<ServoRig*>(arg);
    while (true) {
        self->Tick();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void ServoRig::Tick() {
    if (!attached_) return;
    int64_t now = MdeeNowMs();
    for (int i = 0; i < 4; i++) {
        if (!enabled_[i]) continue;
        if (moving_[i]) {
            float t = (move_ms_[i] <= 0) ? 1.0f : (float)(now - move_start_[i]) / (float)move_ms_[i];
            if (t >= 1.0f) {
                base_[i] = target_[i];
                moving_[i] = false;
            } else {
                if (t < 0.0f) t = 0.0f;
                float s = t * t * (3.0f - 2.0f * t);
                base_[i] = from_[i] + (target_[i] - from_[i]) * s;
            }
        }
        float pose = base_[i];
        if (wiggle_amp_[i] != 0.0f && now < wiggle_until_[i] && wiggle_period_[i] > 0) {
            float ph = (float)(now - wiggle_start_[i]) / (float)wiggle_period_[i];
            pose += wiggle_amp_[i] * sinf(ph * 6.2831853f);
        } else if (now >= wiggle_until_[i]) {
            wiggle_amp_[i] = 0.0f;
        }
        WriteAngle(i, AngleFor(i, pose));
    }
}
