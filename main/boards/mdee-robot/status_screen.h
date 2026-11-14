#ifndef STATUS_SCREEN_H
#define STATUS_SCREEN_H

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

// 240x240 status panel. Drawn into a PSRAM RGB565 buffer, flushed in strips.
class StatusScreen {
public:
    static StatusScreen& GetInstance();
    void Init(esp_lcd_panel_handle_t panel);
    void SetState(const char* state);
    void SetChat(const char* role, const char* content);
    void ClearChat();
    void SetAlert(const char* text);
    void ClearAlertIf(const char* text);
    void Sleep();

private:
    StatusScreen() = default;
    static void Task(void* arg);
    void Render();
    void Fill(int x, int y, int w, int h, uint16_t color);
    void Glyph(int x, int y, char ch, int scale, uint16_t color);
    void Text(int x, int y, const char* s, int scale, uint16_t color);
    uint16_t* fb_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    char state_[40] = "Boot";
    char alert_[40] = "";
    char chat_[3][24] = {{0}, {0}, {0}};
    int chat_n_ = 0;
    bool asleep_ = false;
    bool inited_ = false;
};

#endif
