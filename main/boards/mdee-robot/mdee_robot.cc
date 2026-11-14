#include "mdee_robot.h"
#include "config.h"
#include "mdee_util.h"
#include "pack_battery.h"
#include "performer.h"

#include <driver/gpio.h>
#include <driver/mcpwm_prelude.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <algorithm>
#include <cmath>

#define TAG "MdeeRobot"

static void ForceGen(mcpwm_gen_handle_t gen, int level) {
    if (gen == nullptr) return;
    mcpwm_generator_set_force_level(gen, level, true);
}

MdeeRobot& MdeeRobot::GetInstance() {
    static MdeeRobot instance;
    return instance;
}

int MdeeRobot::SpeedPct(MdeeSpeed s) {
    switch (s) {
        case MdeeSpeed::kSlow: return SPEED_SLOW_PCT;
        case MdeeSpeed::kMedium: return SPEED_MEDIUM_PCT;
        case MdeeSpeed::kFast: return SPEED_FAST_PCT;
        case MdeeSpeed::kTurbo: return SPEED_TURBO_PCT;
        default: return SPEED_MEDIUM_PCT;
    }
}

bool MdeeRobot::ParseSpeed(const std::string& n, MdeeSpeed& out) {
    if (n == "slow") out = MdeeSpeed::kSlow;
    else if (n == "medium" || n == "normal") out = MdeeSpeed::kMedium;
    else if (n == "fast") out = MdeeSpeed::kFast;
    else if (n == "turbo" || n == "high" || n == "high_speed") out = MdeeSpeed::kTurbo;
    else return false;
    return true;
}

bool MdeeRobot::ParseDir(const std::string& n, MdeeDir& out) {
    if (n == "forward") out = MdeeDir::kForward;
    else if (n == "backward" || n == "back" || n == "reverse") out = MdeeDir::kBackward;
    else if (n == "left") out = MdeeDir::kLeft;
    else if (n == "right") out = MdeeDir::kRight;
    else if (n == "stop") out = MdeeDir::kStop;
    else return false;
    return true;
}

const char* MdeeRobot::ModeName(MdeeMode m) {
    switch (m) {
        case MdeeMode::kManual: return "manual";
        case MdeeMode::kWander: return "wander";
        case MdeeMode::kFollow: return "follow";
        default: return "manual";
    }
}

bool MdeeRobot::ParseMode(const std::string& n, MdeeMode& out) {
    if (n == "manual" || n == "idle") out = MdeeMode::kManual;
    else if (n == "wander" || n == "explore") out = MdeeMode::kWander;
    else if (n == "follow") out = MdeeMode::kFollow;
    else return false;
    return true;
}

MdeeMode MdeeRobot::GetMode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

void MdeeRobot::InitMotors() {
    const gpio_num_t pins[4] = {MOTOR_AIN1_PIN, MOTOR_AIN2_PIN, MOTOR_BIN1_PIN, MOTOR_BIN2_PIN};
    gpio_config_t io = {};
    io.mode = GPIO_MODE_OUTPUT;
    io.pin_bit_mask = (1ULL << pins[0]) | (1ULL << pins[1]) | (1ULL << pins[2]) | (1ULL << pins[3]);
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    for (int i = 0; i < 4; i++) gpio_set_level(pins[i], 0);

    mcpwm_timer_handle_t timer = nullptr;
    mcpwm_timer_config_t tc = {};
    tc.group_id = 0;
    tc.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    tc.resolution_hz = 10000000;
    tc.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    tc.period_ticks = 10000000 / MOTOR_PWM_HZ;
    period_ticks_ = (int)tc.period_ticks;
    if (mcpwm_new_timer(&tc, &timer) != ESP_OK) {
        ESP_LOGE(TAG, "MCPWM timer failed");
        return;
    }
    timer_ = timer;

    mcpwm_oper_handle_t opers[2] = {nullptr, nullptr};
    for (int o = 0; o < 2; o++) {
        mcpwm_operator_config_t oc = {};
        oc.group_id = 0;
        if (mcpwm_new_operator(&oc, &opers[o]) != ESP_OK ||
            mcpwm_operator_connect_timer(opers[o], timer) != ESP_OK) {
            ESP_LOGE(TAG, "MCPWM operator failed");
            return;
        }
    }
    for (int i = 0; i < 4; i++) {
        mcpwm_oper_handle_t oper = opers[i / 2];
        mcpwm_cmpr_handle_t cmpr = nullptr;
        mcpwm_comparator_config_t cc = {};
        cc.flags.update_cmp_on_tez = true;
        mcpwm_gen_handle_t gen = nullptr;
        mcpwm_generator_config_t gc = {};
        gc.gen_gpio_num = pins[i];
        if (mcpwm_new_comparator(oper, &cc, &cmpr) != ESP_OK ||
            mcpwm_new_generator(oper, &gc, &gen) != ESP_OK) {
            ESP_LOGE(TAG, "MCPWM generator failed");
            return;
        }
        mcpwm_gen_timer_event_action_t tev = {};
        tev.direction = MCPWM_TIMER_DIRECTION_UP;
        tev.event = MCPWM_TIMER_EVENT_EMPTY;
        tev.action = MCPWM_GEN_ACTION_HIGH;
        mcpwm_generator_set_action_on_timer_event(gen, tev);
        mcpwm_gen_compare_event_action_t cev = {};
        cev.direction = MCPWM_TIMER_DIRECTION_UP;
        cev.comparator = cmpr;
        cev.action = MCPWM_GEN_ACTION_LOW;
        mcpwm_generator_set_action_on_compare_event(gen, cev);
        ForceGen(gen, 0);
        cmpr_[i] = cmpr;
        gen_[i] = gen;
    }
    if (mcpwm_timer_enable(timer) != ESP_OK ||
        mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK) {
        ESP_LOGE(TAG, "MCPWM start failed");
        return;
    }
    motors_ok_ = true;
    ESP_LOGI(TAG, "Motors MCPWM %d Hz, max %.1f V", MOTOR_PWM_HZ, MOTOR_MAX_VOLTS);
}

void MdeeRobot::SetInDuty(int index, float duty) {
    mcpwm_gen_handle_t gen = static_cast<mcpwm_gen_handle_t>(gen_[index]);
    mcpwm_cmpr_handle_t cmpr = static_cast<mcpwm_cmpr_handle_t>(cmpr_[index]);
    if (gen == nullptr || cmpr == nullptr) return;
    if (duty <= 0.001f) {
        ForceGen(gen, 0);
        return;
    }
    if (duty > 1.0f) duty = 1.0f;
    uint32_t ticks = (uint32_t)(duty * (float)(period_ticks_ - 1));
    if (ticks < 1) ticks = 1;
    if (ticks >= (uint32_t)period_ticks_) ticks = (uint32_t)period_ticks_ - 1;
    ForceGen(gen, -1);
    mcpwm_comparator_set_compare_value(cmpr, ticks);
}

void MdeeRobot::ApplyMotor(int left_pct, int right_pct) {
    if (!motors_ok_) return;
    if (MOTOR_LEFT_INVERT) left_pct = -left_pct;
    if (MOTOR_RIGHT_INVERT) right_pct = -right_pct;
    float pack = PackBattery::GetInstance().VoltsOrAssumed();
    if (pack < 0.5f) pack = MOTOR_ASSUME_PACK_V;
    auto duty_for = [&](int pct) -> float {
        int mag = pct < 0 ? -pct : pct;
        if (mag == 0) return 0.0f;
        if (mag > 100) mag = 100;
        float volts = MOTOR_MIN_START_VOLTS +
                      (MOTOR_MAX_VOLTS - MOTOR_MIN_START_VOLTS) * ((float)mag / 100.0f);
        if (volts > MOTOR_MAX_VOLTS) volts = MOTOR_MAX_VOLTS;
        float duty = volts / pack;
        if (duty > 1.0f) duty = 1.0f;
        if (duty < 0.0f) duty = 0.0f;
        return duty;
    };
    float ld = duty_for(left_pct);
    float rd = duty_for(right_pct);
    // Positive percent: IN1 PWM, IN2 low. Negative: the other way. Zero: both forced low.
    if (left_pct > 0) {
        SetInDuty(0, ld);
        SetInDuty(1, 0);
    } else if (left_pct < 0) {
        SetInDuty(0, 0);
        SetInDuty(1, ld);
    } else {
        SetInDuty(0, 0);
        SetInDuty(1, 0);
    }
    if (right_pct > 0) {
        SetInDuty(2, rd);
        SetInDuty(3, 0);
    } else if (right_pct < 0) {
        SetInDuty(2, 0);
        SetInDuty(3, rd);
    } else {
        SetInDuty(2, 0);
        SetInDuty(3, 0);
    }
}

void MdeeRobot::InitUltrasonic() {
    gpio_config_t out = {};
    out.mode = GPIO_MODE_OUTPUT;
    out.pin_bit_mask = 1ULL << US_TRIG_PIN;
    out.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&out);
    gpio_set_level(US_TRIG_PIN, 0);

    gpio_config_t in = {};
    in.mode = GPIO_MODE_INPUT;
    in.pull_down_en = GPIO_PULLDOWN_ENABLE;
    in.pull_up_en = GPIO_PULLUP_DISABLE;
    in.intr_type = GPIO_INTR_DISABLE;
    in.pin_bit_mask = 1ULL << US_ECHO_FRONT_PIN;
    gpio_config(&in);
    in.pin_bit_mask = 1ULL << US_ECHO_REAR_PIN;
    in.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&in);
}

void MdeeRobot::MeasurePair(int& front, int& rear) {
    gpio_set_level(US_TRIG_PIN, 0);
    esp_rom_delay_us(4);
    gpio_set_level(US_TRIG_PIN, 1);
    esp_rom_delay_us(10);
    gpio_set_level(US_TRIG_PIN, 0);
    int64_t t0 = esp_timer_get_time();
    int64_t rise_f = 0, rise_r = 0;
    bool seen_f = false, seen_r = false, done_f = false, done_r = false;
    int64_t fall_f = 0, fall_r = 0;
    while (!done_f || !done_r) {
        int64_t now = esp_timer_get_time();
        if (!done_f) {
            int lv = gpio_get_level(US_ECHO_FRONT_PIN);
            if (!seen_f) {
                if (lv) {
                    seen_f = true;
                    rise_f = now;
                } else if (now - t0 > US_ECHO_WAIT_US) {
                    done_f = true;
                }
            } else if (!lv) {
                done_f = true;
                fall_f = now;
            } else if (now - rise_f > US_ECHO_HIGH_US) {
                done_f = true;
            }
        }
        if (!done_r) {
            int lv = gpio_get_level(US_ECHO_REAR_PIN);
            if (!seen_r) {
                if (lv) {
                    seen_r = true;
                    rise_r = now;
                } else if (now - t0 > US_ECHO_WAIT_US) {
                    done_r = true;
                }
            } else if (!lv) {
                done_r = true;
                fall_r = now;
            } else if (now - rise_r > US_ECHO_HIGH_US) {
                done_r = true;
            }
        }
        if (now - t0 > 30000) break;
    }
    auto to_cm = [](int64_t rise, int64_t fall) -> int {
        if (fall <= rise) return US_NOTHING_CM;
        int cm = (int)((fall - rise) / 58);
        if (cm < 2 || cm > 350) return US_NOTHING_CM;
        return cm;
    };
    front = to_cm(rise_f, fall_f);
    rear = to_cm(rise_r, fall_r);
}

void MdeeRobot::Init() {
    if (inited_) return;
    inited_ = true;
    InitMotors();
    InitUltrasonic();
    xTaskCreate(&MdeeRobot::ControlTask, "mdee_ctl", 4096, this, 8, nullptr);
    xTaskCreate(&MdeeRobot::SensorTask, "mdee_sns", 4096, this, 5, nullptr);
}

void MdeeRobot::Move(MdeeDir dir, MdeeSpeed speed, int duration_ms) {
    int s = SpeedPct(speed);
    switch (dir) {
        case MdeeDir::kForward: Drive(s, s, duration_ms); break;
        case MdeeDir::kBackward: Drive(-s, -s, duration_ms); break;
        case MdeeDir::kLeft: Drive(-s, s, duration_ms); break;
        case MdeeDir::kRight: Drive(s, -s, duration_ms); break;
        case MdeeDir::kStop: Stop(); break;
        default: Stop(); break;
    }
}

void MdeeRobot::Drive(int left_pct, int right_pct, int hold_ms) {
    PerformerCancelFromOutside();
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = MdeeMode::kManual;
    cmd_left_ = MdeeClamp(left_pct, -100, 100);
    cmd_right_ = MdeeClamp(right_pct, -100, 100);
    int hold = hold_ms > 0 ? hold_ms : DRIVE_DEADMAN_MS;
    hold = MdeeClamp(hold, 50, 60000);
    drive_deadline_ms_ = MdeeNowMs() + hold;
}

void MdeeRobot::HaltMotion() {
    std::lock_guard<std::mutex> lock(mutex_);
    mode_ = MdeeMode::kManual;
    cmd_left_ = 0;
    cmd_right_ = 0;
    drive_deadline_ms_ = 0;
    wander_state_ = W::kCruise;
}

void MdeeRobot::Stop() {
    PerformerCancelFromOutside();
    HaltMotion();
}

void MdeeRobot::SetMode(MdeeMode mode) {
    PerformerCancelFromOutside();
    std::lock_guard<std::mutex> lock(mutex_);
    if (mode != MdeeMode::kManual && PackBattery::GetInstance().Cutoff()) mode = MdeeMode::kManual;
    mode_ = mode;
    cmd_left_ = 0;
    cmd_right_ = 0;
    drive_deadline_ms_ = 0;
    wander_state_ = W::kCruise;
    avoid_count_ = 0;
}

void MdeeRobot::SensorTask(void* arg) {
    auto* self = static_cast<MdeeRobot*>(arg);
    int fbuf[3] = {US_NOTHING_CM, US_NOTHING_CM, US_NOTHING_CM};
    int rbuf[3] = {US_NOTHING_CM, US_NOTHING_CM, US_NOTHING_CM};
    int idx = 0;
    while (true) {
        int f = US_NOTHING_CM, r = US_NOTHING_CM;
        self->MeasurePair(f, r);
        fbuf[idx] = f;
        rbuf[idx] = r;
        idx = (idx + 1) % 3;
        int fs[3] = {fbuf[0], fbuf[1], fbuf[2]};
        int rs[3] = {rbuf[0], rbuf[1], rbuf[2]};
        std::sort(fs, fs + 3);
        std::sort(rs, rs + 3);
        self->front_cm_ = fs[1];
        self->rear_cm_ = rs[1];
        vTaskDelay(pdMS_TO_TICKS(40));
    }
}

void MdeeRobot::ControlTask(void* arg) {
    auto* self = static_cast<MdeeRobot*>(arg);
    while (true) {
        self->ControlStep();
        vTaskDelay(pdMS_TO_TICKS(MOTOR_CONTROL_MS));
    }
}

void MdeeRobot::WanderStep(int64_t now) {
    const int cruise = SpeedPct(MdeeSpeed::kMedium);
    int front = front_cm_;
    int rear = rear_cm_;
    switch (wander_state_) {
        case W::kCruise:
            cmd_left_ = cruise;
            cmd_right_ = cruise;
            if (front < WANDER_FRONT_CM) {
                if (now - avoid_window_start_ms_ > 6000) {
                    avoid_window_start_ms_ = now;
                    avoid_count_ = 0;
                }
                avoid_count_++;
                wander_turn_dir_ = (esp_random() & 1) ? 1 : -1;
                if (rear > WANDER_REAR_CLEAR_CM) {
                    wander_state_ = W::kBackup;
                    wander_until_ms_ = now + (avoid_count_ >= 3 ? 900 : 450);
                } else {
                    wander_state_ = W::kTurn;
                    wander_until_ms_ = now + 450 + (int)(esp_random() % 650);
                }
            }
            break;
        case W::kBackup:
            cmd_left_ = -SpeedPct(MdeeSpeed::kSlow);
            cmd_right_ = cmd_left_;
            if (now > wander_until_ms_ || rear < OBSTACLE_STOP_CM + 4) {
                wander_state_ = W::kTurn;
                int extra = avoid_count_ >= 3 ? 400 : 0;
                wander_until_ms_ = now + 450 + (int)(esp_random() % 650) + extra;
            }
            break;
        case W::kTurn: {
            int s = SpeedPct(MdeeSpeed::kMedium);
            cmd_left_ = -s * wander_turn_dir_;
            cmd_right_ = s * wander_turn_dir_;
            if (now > wander_until_ms_) wander_state_ = W::kCruise;
            break;
        }
        default:
            wander_state_ = W::kCruise;
            break;
    }
}

void MdeeRobot::FollowStep() {
    int d = front_cm_;
    if (d >= FOLLOW_FAR_CM) {
        cmd_left_ = 0;
        cmd_right_ = 0;
    } else if (d > FOLLOW_GO_CM) {
        int s = d > 65 ? SPEED_MEDIUM_PCT : SPEED_SLOW_PCT;
        cmd_left_ = s;
        cmd_right_ = s;
    } else if (d < FOLLOW_CLOSE_CM && rear_cm_ > OBSTACLE_STOP_CM + 8) {
        cmd_left_ = -SPEED_SLOW_PCT;
        cmd_right_ = -SPEED_SLOW_PCT;
    } else {
        cmd_left_ = 0;
        cmd_right_ = 0;
    }
}

void MdeeRobot::ControlStep() {
    int64_t now = MdeeNowMs();
    int tl = 0;
    int tr = 0;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (mode_ == MdeeMode::kWander) WanderStep(now);
        else if (mode_ == MdeeMode::kFollow) FollowStep();
        else if (drive_deadline_ms_ != 0 && now > drive_deadline_ms_) {
            cmd_left_ = 0;
            cmd_right_ = 0;
            drive_deadline_ms_ = 0;
        }
        tl = cmd_left_;
        tr = cmd_right_;
    }

    int front = front_cm_;
    int rear = rear_cm_;
    bool pivot = (tl * tr) < 0;
    if (!pivot) {
        if (tl + tr > 0) {
            if (front < OBSTACLE_STOP_CM) {
                tl = 0;
                tr = 0;
            } else if (front < OBSTACLE_SLOW_CM) {
                int span = OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM;
                int pct = 40;
                if (span > 0) pct = 40 + 60 * (front - OBSTACLE_STOP_CM) / span;
                tl = tl * pct / 100;
                tr = tr * pct / 100;
            }
        } else if (tl + tr < 0) {
            if (rear < OBSTACLE_STOP_CM) {
                tl = 0;
                tr = 0;
            } else if (rear < OBSTACLE_SLOW_CM) {
                int span = OBSTACLE_SLOW_CM - OBSTACLE_STOP_CM;
                int pct = 40;
                if (span > 0) pct = 40 + 60 * (rear - OBSTACLE_STOP_CM) / span;
                tl = tl * pct / 100;
                tr = tr * pct / 100;
            }
        }
    }
    if (PackBattery::GetInstance().Cutoff()) {
        tl = 0;
        tr = 0;
    }

    auto ramp = [](int cur, int target) {
        if (target == 0) return 0;
        if (target > cur) return std::min(cur + MOTOR_RAMP_PCT, target);
        return std::max(cur - MOTOR_RAMP_PCT, target);
    };
    int nl = ramp(applied_left_, tl);
    int nr = ramp(applied_right_, tr);
    applied_left_ = nl;
    applied_right_ = nr;
    out_left_ = nl;
    out_right_ = nr;
    ApplyMotor(nl, nr);
}
