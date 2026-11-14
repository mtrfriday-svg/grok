#ifndef LIGHT_RIG_H
#define LIGHT_RIG_H

#include <cstdint>

enum class BarEffect {
    kOff, kSolid, kBreathe, kRainbow, kHeartbeat, kFlash, kChase, kSparkle
};

enum class BlueMode {
    kOff, kOn, kBreathe, kBlink, kPulse, kAuto
};

class LightRig {
public:
    static LightRig& GetInstance();
    void Init();

    bool ParseEffect(const char* name, BarEffect& out) const;
    bool ParseBlue(const char* name, BlueMode& out) const;

    // which: "left", "right", "both"
    void SetBars(const char* which, BarEffect effect, uint8_t r, uint8_t g, uint8_t b);
    void SetBarsAuto(bool on);
    void SetBlue(BlueMode mode);
    void SetBrightness(const char* target, int percent);
    int BarBrightness() const { return bar_bright_; }
    int BlueBrightness() const { return blue_bright_; }

    void ApplyStatus(const char* status);
    void SetThinking(bool on);
    void SetLowBattery(bool on);
    void SetPerformanceRainbow(bool on);
    void AllOff();

private:
    LightRig() = default;
    static void Task(void* arg);
    void Tick();
    void PaintBar(int bar, int64_t now);
    void PaintBlue(int64_t now);
    static void RgbFromHsv(int h, uint8_t& r, uint8_t& g, uint8_t& b);

    bool inited_ = false;
    void* strip_ = nullptr;
    bool bars_manual_ = false;
    bool blue_manual_ = false;
    bool thinking_ = false;
    bool low_battery_ = false;
    bool perf_rainbow_ = false;
    BarEffect effect_[2] = {BarEffect::kBreathe, BarEffect::kBreathe};
    uint8_t cr_[2] = {20, 20};
    uint8_t cg_[2] = {216, 216};
    uint8_t cb_[2] = {255, 255};
    BlueMode blue_ = BlueMode::kAuto;
    int bar_bright_ = LIGHT_DEFAULT_BRIGHTNESS;
    int blue_bright_ = LIGHT_DEFAULT_BRIGHTNESS;
    int auto_kind_ = 0;   // 0 standby, 1 listen, 2 speak, 3 connect
};

#endif
