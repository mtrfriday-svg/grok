#ifndef SOUND_FX_H
#define SOUND_FX_H

// Short original tones. Play() only enqueues; a low-priority task renders them.
enum class Sfx {
    kStartup, kWake, kHappy, kSad, kSurprise, kYawn, kPurr,
    kCry, kHmm, kAck, kError, kLowBat, kShutdown
};

class SoundFx {
public:
    static SoundFx& GetInstance();
    void Init();
    void Play(Sfx id);

private:
    SoundFx() = default;
    static void Task(void* arg);
    void Render(Sfx id);

    void* queue_ = nullptr;
};

#endif
