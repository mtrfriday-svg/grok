#include "status_screen.h"
#include "config.h"
#include "mdee_net.h"
#include "mdee_robot.h"
#include "mdee_util.h"
#include "mood_engine.h"
#include "oled_font6x8.h"
#include "pack_battery.h"
#include "performer.h"

#include <esp_heap_caps.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstring>
#include <mutex>

#define TAG "Status"

static std::mutex g_status_mu;

static uint16_t Rgb(uint8_t r, uint8_t g, uint8_t b) {
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

StatusScreen& StatusScreen::GetInstance() {
    static StatusScreen instance;
    return instance;
}

void StatusScreen::Init(esp_lcd_panel_handle_t panel) {
    if (inited_) return;
    panel_ = panel;
    size_t bytes = (size_t)STATUS_WIDTH * (size_t)STATUS_HEIGHT * sizeof(uint16_t);
    fb_ = static_cast<uint16_t*>(heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (fb_ == nullptr) {
        fb_ = static_cast<uint16_t*>(heap_caps_aligned_alloc(64, bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    }
    if (fb_ == nullptr) {
        ESP_LOGE(TAG, "Framebuffer alloc failed");
        return;
    }
    memset(fb_, 0, bytes);
    inited_ = true;
    xTaskCreate(&StatusScreen::Task, "status", 4096, this, 2, nullptr);
}

void StatusScreen::SetState(const char* state) {
    std::lock_guard<std::mutex> lock(g_status_mu);
    MdeeCopy(state_, sizeof(state_), state != nullptr ? state : "");
}

void StatusScreen::SetAlert(const char* text) {
    std::lock_guard<std::mutex> lock(g_status_mu);
    MdeeCopy(alert_, sizeof(alert_), text != nullptr ? text : "");
}

void StatusScreen::ClearAlertIf(const char* text) {
    if (text == nullptr) return;
    std::lock_guard<std::mutex> lock(g_status_mu);
    if (strcmp(alert_, text) == 0) alert_[0] = '\0';
}

void StatusScreen::ClearChat() {
    std::lock_guard<std::mutex> lock(g_status_mu);
    chat_n_ = 0;
    for (int i = 0; i < 3; i++) chat_[i][0] = '\0';
}

void StatusScreen::SetChat(const char* role, const char* content) {
    if (role != nullptr && strcmp(role, "system") == 0) return;
    char raw[160];
    raw[0] = '\0';
    const char* prefix = "M ";
    if (role != nullptr && (strcmp(role, "user") == 0 || strcmp(role, "User") == 0)) prefix = "U ";
    MdeeCopy(raw, sizeof(raw), prefix);
    size_t used = 0;
    while (raw[used] != '\0') used++;
    if (content != nullptr) {
        for (size_t i = 0; content[i] != '\0' && used + 1 < sizeof(raw); i++) {
            unsigned char c = (unsigned char)content[i];
            if (c < 32) c = ' ';
            else if (c > 126) c = '?';
            raw[used++] = (char)c;
        }
        raw[used] = '\0';
    }
    char lines[6][24];
    int nlines = 0;
    size_t i = 0;
    size_t len = 0;
    while (raw[len] != '\0') len++;
    while (i < len && nlines < 6) {
        size_t take = len - i;
        if (take > 20) {
            take = 20;
            for (size_t k = 20; k > 8; k--) {
                if (raw[i + k] == ' ') {
                    take = k;
                    break;
                }
            }
        }
        size_t c = 0;
        for (; c < take && c + 1 < sizeof(lines[0]); c++) lines[nlines][c] = raw[i + c];
        lines[nlines][c] = '\0';
        nlines++;
        i += take;
        while (i < len && raw[i] == ' ') i++;
    }
    std::lock_guard<std::mutex> lock(g_status_mu);
    for (int li = 0; li < nlines; li++) {
        if (chat_n_ < 3) {
            MdeeCopy(chat_[chat_n_], sizeof(chat_[chat_n_]), lines[li]);
            chat_n_++;
        } else {
            MdeeCopy(chat_[0], sizeof(chat_[0]), chat_[1]);
            MdeeCopy(chat_[1], sizeof(chat_[1]), chat_[2]);
            MdeeCopy(chat_[2], sizeof(chat_[2]), lines[li]);
        }
    }
}

void StatusScreen::Sleep() {
    asleep_ = true;
    vTaskDelay(pdMS_TO_TICKS(80));
    if (panel_ != nullptr) {
        esp_lcd_panel_disp_on_off(panel_, false);
        esp_lcd_panel_disp_sleep(panel_, true);
    }
}

void StatusScreen::Fill(int x, int y, int w, int h, uint16_t color) {
    if (fb_ == nullptr) return;
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > STATUS_WIDTH) w = STATUS_WIDTH - x;
    if (y + h > STATUS_HEIGHT) h = STATUS_HEIGHT - y;
    if (w <= 0 || h <= 0) return;
    for (int row = 0; row < h; row++) {
        uint16_t* p = fb_ + (y + row) * STATUS_WIDTH + x;
        for (int col = 0; col < w; col++) p[col] = color;
    }
}

void StatusScreen::Glyph(int x, int y, char ch, int scale, uint16_t color) {
    if (scale < 1) scale = 1;
    unsigned char c = (unsigned char)ch;
    if (c < 32 || c > 126) c = '?';
    const uint8_t* col = kOledFont6x8[c - 32];
    for (int cx = 0; cx < 6; cx++) {
        uint8_t bits = col[cx];
        for (int cy = 0; cy < 8; cy++) {
            if ((bits & (1u << cy)) == 0) continue;
            Fill(x + cx * scale, y + cy * scale, scale, scale, color);
        }
    }
}

void StatusScreen::Text(int x, int y, const char* s, int scale, uint16_t color) {
    if (s == nullptr) return;
    int cursor = x;
    for (int i = 0; s[i] != '\0'; i++) {
        if (cursor > STATUS_WIDTH - 6 * scale) break;
        Glyph(cursor, y, s[i], scale, color);
        cursor += 6 * scale;
    }
}

static int WifiBars() {
    int rssi = MdeeNet::Rssi();
    if (rssi >= -55) return 4;
    if (rssi >= -65) return 3;
    if (rssi >= -75) return 2;
    if (rssi >= -85) return 1;
    return 0;
}

void StatusScreen::Render() {
    if (fb_ == nullptr || panel_ == nullptr || asleep_) return;
    char state[40];
    char alert[40];
    char chat[3][24];
    int chat_n = 0;
    {
        std::lock_guard<std::mutex> lock(g_status_mu);
        MdeeCopy(state, sizeof(state), state_);
        MdeeCopy(alert, sizeof(alert), alert_);
        chat_n = chat_n_;
        for (int i = 0; i < 3; i++) MdeeCopy(chat[i], sizeof(chat[i]), chat_[i]);
    }
    const uint16_t bg = Rgb(0, 0, 0);
    const uint16_t fg = Rgb(230, 245, 255);
    const uint16_t cyan = Rgb(20, 216, 255);
    const uint16_t dim = Rgb(120, 140, 160);
    memset(fb_, 0, (size_t)STATUS_WIDTH * STATUS_HEIGHT * sizeof(uint16_t));

    Text(4, 4, "M'DEE", 2, cyan);
    int bars = WifiBars();
    for (int b = 0; b < 4; b++) {
        int h = 4 + b * 3;
        uint16_t col = (b < bars) ? cyan : Rgb(30, 40, 50);
        Fill(168 + b * 14, 18 - h, 8, h, col);
    }
    Text(4, 26, state, 2, fg);

    auto& bat = PackBattery::GetInstance();
    int pct = bat.Connected() ? bat.Percent() : 0;
    uint16_t bat_col = Rgb(40, 220, 90);
    if (!bat.Connected()) bat_col = dim;
    else if (pct < 15) bat_col = Rgb(230, 40, 40);
    else if (pct <= 40) bat_col = Rgb(230, 190, 40);

    char pct_buf[16];
    pct_buf[0] = '\0';
    if (!bat.Connected()) {
        MdeeCopy(pct_buf, sizeof(pct_buf), "--");
    } else {
        // Small integer, buffer is larger than any 3-digit value plus a sign and percent.
        int v = MdeeClamp(pct, 0, 100);
        pct_buf[0] = '0' + (v / 100) % 10;
        int n = 0;
        if (v >= 100) pct_buf[n++] = '1';
        if (v >= 10) pct_buf[n++] = '0' + ((v / 10) % 10);
        pct_buf[n++] = '0' + (v % 10);
        pct_buf[n++] = '%';
        pct_buf[n] = '\0';
        (void)pct_buf[0];
    }
    Text(4, 50, pct_buf, 4, bat_col);

    char volt[24];
    volt[0] = '\0';
    if (bat.Connected()) {
        int mv = (int)(bat.Volts() * 100.0f + 0.5f);
        if (mv < 0) mv = 0;
        if (mv > 9999) mv = 9999;
        int whole = mv / 100;
        int frac = mv % 100;
        volt[0] = '0' + (whole / 10) % 10;
        volt[1] = '0' + whole % 10;
        volt[2] = '.';
        volt[3] = '0' + (frac / 10) % 10;
        volt[4] = '0' + frac % 10;
        volt[5] = 'V';
        volt[6] = '\0';
        if (whole < 10) {
            volt[0] = '0' + whole;
            volt[1] = '.';
            volt[2] = '0' + (frac / 10) % 10;
            volt[3] = '0' + frac % 10;
            volt[4] = 'V';
            volt[5] = '\0';
        }
    } else {
        MdeeCopy(volt, sizeof(volt), "no pack");
    }
    Text(120, 62, volt, 2, fg);

    Fill(8, 96, 224, 12, Rgb(20, 28, 36));
    int bar_w = bat.Connected() ? (224 * MdeeClamp(pct, 0, 100)) / 100 : 0;
    Fill(8, 96, bar_w, 12, bat_col);

    auto& robot = MdeeRobot::GetInstance();
    char dist[40];
    int front = robot.FrontCm();
    int rear = robot.RearCm();
    auto put3 = [](char* d, int v) {
        if (v > 999) v = 999;
        if (v < 0) v = 0;
        d[0] = '0' + (v / 100) % 10;
        d[1] = '0' + (v / 10) % 10;
        d[2] = '0' + v % 10;
        d[3] = '\0';
    };
    char fb[4], rb[4];
    put3(fb, front);
    put3(rb, rear);
    MdeeCopy(dist, sizeof(dist), "F ");
    MdeeAppend(dist, sizeof(dist), fb);
    MdeeAppend(dist, sizeof(dist), " R ");
    MdeeAppend(dist, sizeof(dist), rb);
    Text(4, 114, dist, 2, fg);
    int fw = front >= US_NOTHING_CM ? 8 : MdeeClamp(front, 0, 80);
    int rw = rear >= US_NOTHING_CM ? 8 : MdeeClamp(rear, 0, 80);
    Fill(150, 116, fw, 8, cyan);
    Fill(150, 128, rw, 8, Rgb(80, 140, 255));

    char mood[48];
    MoodEngine::GetInstance().Format(mood, sizeof(mood));
    Text(4, 138, mood, 2, dim);

    bool blink = ((MdeeNowMs() / 400) & 1) != 0;
    if (alert[0] != '\0') {
        if (blink) Text(4, 160, alert, 2, Rgb(255, 60, 60));
    } else {
        char mode_line[40];
        MdeeCopy(mode_line, sizeof(mode_line), MdeeRobot::ModeName(robot.GetMode()));
        MdeeAppend(mode_line, sizeof(mode_line), " ");
        MdeeAppend(mode_line, sizeof(mode_line), MoodEngine::GetInstance().IdleName());
        if (Performer::GetInstance().Running()) {
            MdeeCopy(mode_line, sizeof(mode_line), Performer::GetInstance().Current());
        }
        Text(4, 160, mode_line, 2, fg);
    }

    if (chat_n == 0) {
        char ip[64];
        char url[80];
        MdeeCopy(url, sizeof(url), "http://");
        if (MdeeNet::GetIp(ip, sizeof(ip))) MdeeAppend(url, sizeof(url), ip);
        else MdeeAppend(url, sizeof(url), "...");
        Text(4, 184, url, 2, cyan);
    } else {
        for (int i = 0; i < chat_n && i < 3; i++) {
            Text(4, 184 + i * 18, chat[i], 2, fg);
        }
    }
    (void)bg;

    static uint16_t* dma[2] = {nullptr, nullptr};
    constexpr int kRows = 20;
    constexpr int kStrip = STATUS_WIDTH * kRows;
    if (dma[0] == nullptr) {
        dma[0] = static_cast<uint16_t*>(heap_caps_malloc(kStrip * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        dma[1] = static_cast<uint16_t*>(heap_caps_malloc(kStrip * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    }
    if (dma[0] == nullptr || dma[1] == nullptr) return;
    int flip = 0;
    for (int y = 0; y < STATUS_HEIGHT; y += kRows) {
        int rows = kRows;
        if (y + rows > STATUS_HEIGHT) rows = STATUS_HEIGHT - y;
        uint16_t* dst = dma[flip];
        const uint16_t* src = fb_ + y * STATUS_WIDTH;
        int count = STATUS_WIDTH * rows;
        for (int i = 0; i < count; i++) {
#if STATUS_SWAP_BYTES
            dst[i] = (uint16_t)((src[i] << 8) | (src[i] >> 8));
#else
            dst[i] = src[i];
#endif
        }
        esp_lcd_panel_draw_bitmap(panel_, 0, y, STATUS_WIDTH, y + rows, dst);
        flip ^= 1;
    }
}

void StatusScreen::Task(void* arg) {
    auto* self = static_cast<StatusScreen*>(arg);
    while (true) {
        if (!self->asleep_) self->Render();
        vTaskDelay(pdMS_TO_TICKS(STATUS_REFRESH_MS));
    }
}
