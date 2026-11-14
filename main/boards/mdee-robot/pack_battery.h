#ifndef PACK_BATTERY_H
#define PACK_BATTERY_H

// 2S2P pack on GPIO1. Percent is computed per cell (pack volts / 2).
class PackBattery {
public:
    static PackBattery& GetInstance();
    void Init();

    bool Connected() const { return connected_; }
    float Volts() const;
    float VoltsOrAssumed() const;
    int Percent() const { return percent_; }
    bool Cutoff() const { return cutoff_; }
    bool Low() const { return low_; }

private:
    PackBattery() = default;
    static void Task(void* arg);
    void Step();

    bool inited_ = false;
    bool connected_ = false;
    bool cutoff_ = false;
    bool low_ = false;
    bool off_sent_ = false;
    bool low_announced_ = false;
    int percent_ = 0;
    int mv_ = 0;
    int64_t low_v_since_ = 0;
    int64_t critical_since_ = 0;
    int64_t low_pct_since_ = 0;
};

#endif
