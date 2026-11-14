#ifndef SERVO_RIG_H
#define SERVO_RIG_H

#include <cstdint>

// Four positional servos in pose space 0-100. Disabled entries in config.h are skipped.
class ServoRig {
public:
    static ServoRig& GetInstance();
    void Init();

    bool Enabled(int index) const;
    bool Attached() const { return attached_; }
    float Pose(int index) const;

    // Ease-in-out. Duration is extended so SERVO_*_MAX_DPS is never exceeded.
    void MoveTo(int index, float pose, int duration_ms);
    void MoveEars(float left, float right, int duration_ms);
    void MoveNeck(float turn, float tilt, int duration_ms);

    // Overlay wobble in pose units. Does not change the base target.
    void Wiggle(int index, float amplitude, int period_ms, int duration_ms);
    void WiggleEars(float amplitude, int period_ms, int duration_ms);

    // true: PWM starts at the DOWN pose (ears 0, tilt 0, turn centred). false: ledc_stop, limp.
    void Attach(bool on);

    void Perk(int side, int duration_ms);    // side: 0 both, 1 left, 2 right
    void Droop(int side, int duration_ms);
    void Neutral(int side, int duration_ms);
    void Flick(int side);

private:
    ServoRig() = default;
    static void Task(void* arg);
    void Tick();
    int AngleFor(int index, float pose) const;
    void WriteAngle(int index, int angle);
    void BindChannel(int index);
    float DownPose(int index) const;

    bool inited_ = false;
    bool attached_ = false;
    bool enabled_[4] = {false, false, false, false};
    int pin_[4] = {-1, -1, -1, -1};
    int at0_[4] = {90, 90, 90, 90};
    int at100_[4] = {90, 90, 90, 90};
    int max_dps_[4] = {120, 120, 300, 300};
    float base_[4] = {50, 0, 0, 0};
    float from_[4] = {50, 0, 0, 0};
    float target_[4] = {50, 0, 0, 0};
    int64_t move_start_[4] = {0, 0, 0, 0};
    int move_ms_[4] = {0, 0, 0, 0};
    bool moving_[4] = {false, false, false, false};
    float wiggle_amp_[4] = {0, 0, 0, 0};
    int wiggle_period_[4] = {0, 0, 0, 0};
    int64_t wiggle_start_[4] = {0, 0, 0, 0};
    int64_t wiggle_until_[4] = {0, 0, 0, 0};
};

#endif
