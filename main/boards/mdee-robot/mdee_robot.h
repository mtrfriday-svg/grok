#ifndef MDEE_ROBOT_H
#define MDEE_ROBOT_H

#include <atomic>
#include <mutex>
#include <string>

enum class MdeeDir { kForward, kBackward, kLeft, kRight, kStop };
enum class MdeeSpeed { kSlow, kMedium, kFast, kTurbo };
enum class MdeeMode { kManual, kWander, kFollow };

class MdeeRobot {
public:
    static MdeeRobot& GetInstance();
    void Init();

    void Move(MdeeDir dir, MdeeSpeed speed, int duration_ms);
    void Drive(int left_pct, int right_pct, int hold_ms);
    void Stop();
    void HaltMotion();

    static int SpeedPct(MdeeSpeed s);
    static bool ParseSpeed(const std::string& name, MdeeSpeed& out);
    static bool ParseDir(const std::string& name, MdeeDir& out);

    void SetMode(MdeeMode mode);
    MdeeMode GetMode() const;
    static const char* ModeName(MdeeMode m);
    static bool ParseMode(const std::string& name, MdeeMode& out);

    int FrontCm() const { return front_cm_; }
    int RearCm() const { return rear_cm_; }
    int OutLeft() const { return out_left_; }
    int OutRight() const { return out_right_; }

private:
    MdeeRobot() = default;
    void InitMotors();
    void InitUltrasonic();
    void ApplyMotor(int left_pct, int right_pct);
    void SetInDuty(int index, float duty);
    void MeasurePair(int& front, int& rear);
    void ControlStep();
    void WanderStep(int64_t now);
    void FollowStep();
    static void ControlTask(void* arg);
    static void SensorTask(void* arg);

    mutable std::mutex mutex_;
    std::atomic<int> front_cm_{400};
    std::atomic<int> rear_cm_{400};
    std::atomic<int> out_left_{0};
    std::atomic<int> out_right_{0};
    int cmd_left_ = 0;
    int cmd_right_ = 0;
    int applied_left_ = 0;
    int applied_right_ = 0;
    int64_t drive_deadline_ms_ = 0;
    MdeeMode mode_ = MdeeMode::kManual;

    enum class W { kCruise, kBackup, kTurn };
    W wander_state_ = W::kCruise;
    int64_t wander_until_ms_ = 0;
    int wander_turn_dir_ = 1;
    int avoid_count_ = 0;
    int64_t avoid_window_start_ms_ = 0;

    bool motors_ok_ = false;
    int period_ticks_ = 500;
    void* timer_ = nullptr;
    void* cmpr_[4] = {nullptr, nullptr, nullptr, nullptr};
    void* gen_[4] = {nullptr, nullptr, nullptr, nullptr};
    bool inited_ = false;
};

#endif
