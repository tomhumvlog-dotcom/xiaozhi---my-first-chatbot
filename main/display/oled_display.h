#ifndef OLED_DISPLAY_H
#define OLED_DISPLAY_H

#include "lvgl_display.h"

#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>

#include <cstdint>
#include <string>

class OledDisplay : public LvglDisplay {
private:
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;

    lv_obj_t* top_bar_ = nullptr;
    lv_obj_t* status_bar_ = nullptr;
    lv_obj_t* content_ = nullptr;
    lv_obj_t* content_left_ = nullptr;
    lv_obj_t* content_right_ = nullptr;
    lv_obj_t* container_ = nullptr;
    lv_obj_t* side_bar_ = nullptr;
    lv_obj_t* emotion_label_ = nullptr;
    lv_obj_t* animated_face_ = nullptr;
    lv_obj_t* left_eye_ = nullptr;
    lv_obj_t* right_eye_ = nullptr;
    lv_obj_t* left_pupil_ = nullptr;
    lv_obj_t* right_pupil_ = nullptr;
    lv_obj_t* chat_message_label_ = nullptr;
    lv_timer_t* emotion_timer_ = nullptr;
    std::string current_emotion_ = "neutral";
    uint32_t blink_interval_ms_ = 4000;
    int gaze_offset_x_ = 0;
    bool animated_emotion_ = false;
    bool eyes_closed_ = false;
    bool low_battery_was_visible_ = false;
    bool screen_background_saved_ = false;
    lv_color_t normal_screen_bg_color_{};
    lv_opa_t normal_screen_bg_opa_ = LV_OPA_TRANSP;

    virtual bool Lock(int timeout_ms = 0) override;
    virtual void Unlock() override;

    void SetupUI_128x64();
    void SetupUI_128x32();
    void CreateAnimatedFace(lv_obj_t* parent);
    void UpdateAnimatedFace();
    static void EmotionTimerCallback(lv_timer_t* timer);

public:
    OledDisplay(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_handle_t panel, int width,
                int height, bool mirror_x, bool mirror_y);
    ~OledDisplay();

    virtual void SetupUI() override;
    virtual void SetChatMessage(const char* role, const char* content) override;
    virtual void SetEmotion(const char* emotion) override;
    bool SetEmotionMode(const std::string& mode, uint32_t blink_interval_ms);
    virtual void SetTheme(Theme* theme) override;
    virtual bool IsMonochrome() const override { return true; }
    void SetPowerSaveMode(bool on) override;
};

#endif  // OLED_DISPLAY_H
