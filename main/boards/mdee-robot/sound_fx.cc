#include "sound_fx.h"
#include "config.h"
#include "mdee_util.h"

#include "application.h"

#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include <cmath>
#include <cstdint>
#include <vector>

#define TAG "Sfx"

struct Seg {
    float f0;
    float f1;
    int ms;
    float amp;
    float trem;   // Hz, 0 = none
};

SoundFx& SoundFx::GetInstance() {
    static SoundFx instance;
    return instance;
}

void SoundFx::Init() {
    if (queue_ != nullptr) return;
    queue_ = xQueueCreate(6, sizeof(uint8_t));
    xTaskCreate(&SoundFx::Task, "sfx", 6144, this, 2, nullptr);
}

void SoundFx::Play(Sfx id) {
    if (queue_ == nullptr) return;
    uint8_t v = (uint8_t)id;
    xQueueSend((QueueHandle_t)queue_, &v, 0);
}

void SoundFx::Task(void* arg) {
    auto* self = static_cast<SoundFx*>(arg);
    uint8_t id = 0;
    while (true) {
        if (xQueueReceive((QueueHandle_t)self->queue_, &id, portMAX_DELAY) != pdTRUE) continue;
        auto state = Application::GetInstance().GetDeviceState();
        if (state == kDeviceStateSpeaking && (Sfx)id != Sfx::kAck) continue;
        self->Render((Sfx)id);
    }
}

static void RenderSegs(const Seg* segs, int count) {
    constexpr int kRate = 16000;
    constexpr int kChunk = 320;
    std::vector<int16_t> chunk;
    chunk.resize(kChunk);
    int filled = 0;
    float phase = 0.0f;
    auto flush = [&]() {
        if (filled <= 0) return;
        std::vector<int16_t> part(chunk.begin(), chunk.begin() + filled);
        filled = 0;
        Application::GetInstance().GetAudioService().PushPcmToPlayback(std::move(part));
    };
    for (int s = 0; s < count; s++) {
        const Seg& seg = segs[s];
        int samples = seg.ms * (kRate / 1000);
        if (samples < 1) samples = 1;
        int attack = kRate / 100;
        int release = kRate / 40;
        if (attack > samples / 3) attack = samples / 3;
        if (release > samples / 3) release = samples / 3;
        for (int i = 0; i < samples; i++) {
            if ((i & 127) == 0 && Application::GetInstance().GetDeviceState() == kDeviceStateSpeaking &&
                seg.amp < 0.9f) {
                // Keep shutdown/ack audible; drop decorative tones if speech started.
            }
            float t = (samples <= 1) ? 1.0f : (float)i / (float)(samples - 1);
            float freq = seg.f0 + (seg.f1 - seg.f0) * t;
            if (freq < 30.0f) freq = 30.0f;
            phase += 6.2831853f * freq / (float)kRate;
            if (phase > 6.2831853f) phase -= 6.2831853f;
            float env = 1.0f;
            if (i < attack && attack > 0) env = (float)i / (float)attack;
            else if (i > samples - release && release > 0) env = (float)(samples - i) / (float)release;
            float trem = 1.0f;
            if (seg.trem > 0.5f) trem = 0.55f + 0.45f * sinf(6.2831853f * seg.trem * ((float)i / (float)kRate));
            float sample = sinf(phase) * seg.amp * env * trem;
            if (sample > 0.95f) sample = 0.95f;
            if (sample < -0.95f) sample = -0.95f;
            chunk[filled++] = (int16_t)(sample * 32767.0f);
            if (filled >= kChunk) flush();
        }
    }
    flush();
}

void SoundFx::Render(Sfx id) {
    Seg buf[8];
    int n = 0;
    switch (id) {
        case Sfx::kStartup:
            buf[0] = {523, 523, 90, 0.22f, 0};
            buf[1] = {659, 659, 90, 0.22f, 0};
            buf[2] = {784, 784, 90, 0.24f, 0};
            buf[3] = {1046, 1046, 140, 0.26f, 0};
            n = 4;
            break;
        case Sfx::kWake:
            buf[0] = {620, 1480, 130, 0.24f, 0};
            n = 1;
            break;
        case Sfx::kHappy:
            buf[0] = {880, 880, 50, 0.22f, 0};
            buf[1] = {1175, 1175, 50, 0.22f, 0};
            buf[2] = {880, 880, 50, 0.22f, 0};
            buf[3] = {1318, 1568, 90, 0.24f, 0};
            n = 4;
            break;
        case Sfx::kSad:
            buf[0] = {520, 180, 560, 0.22f, 0};
            n = 1;
            break;
        case Sfx::kSurprise:
            buf[0] = {980, 1760, 90, 0.28f, 0};
            n = 1;
            break;
        case Sfx::kYawn:
            buf[0] = {340, 140, 520, 0.20f, 0};
            buf[1] = {140, 260, 220, 0.16f, 0};
            n = 2;
            break;
        case Sfx::kPurr:
            buf[0] = {78, 92, 700, 0.28f, 14};
            n = 1;
            break;
        case Sfx::kCry:
            buf[0] = {420, 560, 180, 0.22f, 7};
            buf[1] = {560, 380, 180, 0.22f, 7};
            buf[2] = {380, 500, 200, 0.20f, 6};
            n = 3;
            break;
        case Sfx::kHmm:
            buf[0] = {196, 180, 200, 0.18f, 0};
            buf[1] = {0, 0, 40, 0.0f, 0};
            buf[2] = {164, 150, 260, 0.18f, 0};
            n = 3;
            break;
        case Sfx::kAck:
            buf[0] = {880, 880, 70, 0.20f, 0};
            buf[1] = {0, 0, 30, 0.0f, 0};
            buf[2] = {1320, 1320, 90, 0.22f, 0};
            n = 3;
            break;
        case Sfx::kError:
            buf[0] = {110, 90, 220, 0.30f, 30};
            n = 1;
            break;
        case Sfx::kLowBat:
            buf[0] = {440, 440, 140, 0.20f, 0};
            buf[1] = {0, 0, 80, 0.0f, 0};
            buf[2] = {330, 330, 180, 0.20f, 0};
            n = 3;
            break;
        case Sfx::kShutdown:
            buf[0] = {640, 70, 780, 0.24f, 0};
            n = 1;
            break;
        default:
            return;
    }
    // A 0 Hz segment is a short silence.
    for (int i = 0; i < n; i++) {
        if (buf[i].f0 < 1.0f && buf[i].f1 < 1.0f) {
            buf[i].amp = 0.0f;
            buf[i].f0 = 40.0f;
            buf[i].f1 = 40.0f;
        }
    }
    RenderSegs(buf, n);
    (void)TAG;
}
