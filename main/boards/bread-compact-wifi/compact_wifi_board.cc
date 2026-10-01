#include "wifi_board.h"
#include "codecs/no_audio_codec.h"
#include "display/oled_display.h"
#include "system_reset.h"
#include "application.h"
#include "button.h"
#include "config.h"
#include "mcp_server.h"
#include "lamp_controller.h"
#include "led/single_led.h"
#include "assets/lang_config.h"
#include "settings.h"

#include <esp_log.h>
#include <esp_netif_sntp.h>
#include <driver/gpio.h>
#include <driver/i2c_master.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <mutex>
#include <utility>

#ifdef SH1106
#include <esp_lcd_panel_sh1106.h>
#endif

#define TAG "CompactWifiBoard"

class CompactWifiBoard : public WifiBoard {
private:
    i2c_master_bus_handle_t display_i2c_bus_;
    esp_lcd_panel_io_handle_t panel_io_ = nullptr;
    esp_lcd_panel_handle_t panel_ = nullptr;
    Display* display_ = nullptr;
    Button boot_button_;
    Button touch_button_;
    Button volume_up_button_;
    Button volume_down_button_;
    QueueHandle_t motor_command_queue_ = nullptr;
    std::mutex schedule_mutex_;
    bool time_sync_started_ = false;

    enum class MotorAction : uint8_t { Stop, Forward, Backward, TurnAround };

    struct MotorCommand {
        MotorAction action;
        uint32_t duration_ms;
    };

    void InitializeDisplayI2c() {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = (i2c_port_t)0,
            .sda_io_num = DISPLAY_SDA_PIN,
            .scl_io_num = DISPLAY_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .intr_priority = 0,
            .trans_queue_depth = 0,
            .flags = {
                .enable_internal_pullup = 1,
            },
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &display_i2c_bus_));
    }

    void InitializeSsd1306Display() {
        // SSD1306 config
        esp_lcd_panel_io_i2c_config_t io_config = {
            .dev_addr = 0x3C,
            .scl_speed_hz = 400 * 1000,
            .control_phase_bytes = 1,
            .dc_bit_offset = 6,
            .lcd_cmd_bits = 8,
            .lcd_param_bits = 8,
            .on_color_trans_done = nullptr,
            .user_ctx = nullptr,
            .flags = {
                .dc_low_on_data = 0,
                .disable_control_phase = 0,
            },
        };

        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(display_i2c_bus_, &io_config, &panel_io_));

        ESP_LOGI(TAG, "Install SSD1306 driver");
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = GPIO_NUM_NC;
        panel_config.bits_per_pixel = 1;

        esp_lcd_panel_ssd1306_config_t ssd1306_config = {
            .height = static_cast<uint8_t>(DISPLAY_HEIGHT),
        };
        panel_config.vendor_config = &ssd1306_config;

#ifdef SH1106
        ESP_ERROR_CHECK(esp_lcd_new_panel_sh1106(panel_io_, &panel_config, &panel_));
#else
        ESP_ERROR_CHECK(esp_lcd_new_panel_ssd1306(panel_io_, &panel_config, &panel_));
#endif
        ESP_LOGI(TAG, "SSD1306 driver installed");

        // Reset the display
        ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_));
        if (esp_lcd_panel_init(panel_) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize display");
            display_ = new NoDisplay();
            return;
        }
        ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_, false));

        // Set the display to on
        ESP_LOGI(TAG, "Turning display on");
        ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_, true));

        display_ = new OledDisplay(panel_io_, panel_, DISPLAY_WIDTH, DISPLAY_HEIGHT, DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
            app.ToggleChatState();
        });
        touch_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        touch_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });

        volume_up_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() + 10;
            if (volume > 100) {
                volume = 100;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_up_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(100);
            GetDisplay()->ShowNotification(Lang::Strings::MAX_VOLUME);
        });

        volume_down_button_.OnClick([this]() {
            auto codec = GetAudioCodec();
            auto volume = codec->output_volume() - 10;
            if (volume < 0) {
                volume = 0;
            }
            codec->SetOutputVolume(volume);
            GetDisplay()->ShowNotification(Lang::Strings::VOLUME + std::to_string(volume));
        });

        volume_down_button_.OnLongPress([this]() {
            GetAudioCodec()->SetOutputVolume(0);
            GetDisplay()->ShowNotification(Lang::Strings::MUTED);
        });
    }

    // 物联网初始化，逐步迁移到 MCP 协议
    cJSON* LoadScheduleItems() {
        Settings settings("schedule");
        auto stored_items = settings.GetString("items", "[]");
        cJSON* items = cJSON_Parse(stored_items.c_str());
        if (!cJSON_IsArray(items)) {
            cJSON_Delete(items);
            items = cJSON_CreateArray();
        }
        return items;
    }

    bool SaveScheduleItems(cJSON* items) {
        CJsonStringUniquePtr json(cJSON_PrintUnformatted(items));
        if (json == nullptr) {
            return false;
        }
        Settings settings("schedule", true);
        settings.SetString("items", json.get());
        return true;
    }

    bool ParseScheduleTime(const std::string& value, time_t& scheduled_at) {
        if (value.size() != 16) {
            return false;
        }

        int year, month, day, hour, minute;
        if (std::sscanf(value.c_str(), "%4d-%2d-%2d %2d:%2d", &year, &month, &day,
                        &hour, &minute) != 5) {
            return false;
        }
        std::tm scheduled_tm = {};
        scheduled_tm.tm_year = year - 1900;
        scheduled_tm.tm_mon = month - 1;
        scheduled_tm.tm_mday = day;
        scheduled_tm.tm_hour = hour;
        scheduled_tm.tm_min = minute;
        scheduled_tm.tm_isdst = -1;
        scheduled_at = std::mktime(&scheduled_tm);

        std::tm verified_tm = {};
        if (scheduled_at == static_cast<time_t>(-1) ||
            localtime_r(&scheduled_at, &verified_tm) == nullptr) {
            return false;
        }
        return verified_tm.tm_year == year - 1900 && verified_tm.tm_mon == month - 1 &&
               verified_tm.tm_mday == day && verified_tm.tm_hour == hour &&
               verified_tm.tm_min == minute;
    }

    ReturnValue AddScheduleItem(const std::string& title, const std::string& time,
                                const std::string& repeat) {
        std::lock_guard<std::mutex> lock(schedule_mutex_);
        time_t scheduled_at;
        if (title.empty() || !ParseScheduleTime(time, scheduled_at)) {
            return std::string("Use a valid local date and time in YYYY-MM-DD HH:MM format");
        }
        if (repeat != "once" && repeat != "daily" && repeat != "weekly") {
            return std::string("repeat must be once, daily, or weekly");
        }

        CJsonUniquePtr items(LoadScheduleItems());
        if (cJSON_GetArraySize(items.get()) >= 12) {
            return std::string("Schedule is full; delete an item before adding another");
        }

        Settings settings("schedule");
        int id = settings.GetInt("next_id", 1);
        CJsonUniquePtr item(cJSON_CreateObject());
        if (item == nullptr || !cJSON_AddNumberToObject(item.get(), "id", id) ||
            !cJSON_AddStringToObject(item.get(), "title", title.c_str()) ||
            !cJSON_AddStringToObject(item.get(), "time", time.c_str()) ||
            !cJSON_AddStringToObject(item.get(), "repeat", repeat.c_str()) ||
            !cJSON_AddNumberToObject(item.get(), "scheduled_at",
                                     static_cast<double>(scheduled_at)) ||
            !cJSON_AddBoolToObject(item.get(), "notified", false) ||
            !cJSON_AddNumberToObject(item.get(), "last_fired_day", 0)) {
            return std::string("Failed to create schedule item");
        }
        cJSON_AddItemToArray(items.get(), item.release());
        if (!SaveScheduleItems(items.get())) {
            return std::string("Failed to save schedule item");
        }
        Settings write_settings("schedule", true);
        write_settings.SetInt("next_id", id + 1);
        return std::string("Saved schedule item ") + std::to_string(id);
    }

    ReturnValue DeleteScheduleItem(int id) {
        std::lock_guard<std::mutex> lock(schedule_mutex_);
        CJsonUniquePtr items(LoadScheduleItems());
        for (int index = 0; index < cJSON_GetArraySize(items.get()); ++index) {
            cJSON* item = cJSON_GetArrayItem(items.get(), index);
            cJSON* item_id = cJSON_GetObjectItem(item, "id");
            if (cJSON_IsNumber(item_id) && item_id->valueint == id) {
                cJSON_DeleteItemFromArray(items.get(), index);
                return SaveScheduleItems(items.get()) ? ReturnValue(true)
                                                      : ReturnValue(std::string("Failed to save schedule list"));
            }
        }
        return std::string("Schedule item not found");
    }

    void CheckScheduleReminders() {
        std::lock_guard<std::mutex> lock(schedule_mutex_);
        const time_t now = std::time(nullptr);
        if (now < 1735689600) {  // Ignore the unsynchronized clock before 2025.
            return;
        }

        std::tm now_tm = {};
        if (localtime_r(&now, &now_tm) == nullptr) {
            return;
        }
        const int today = (now_tm.tm_year + 1900) * 10000 + (now_tm.tm_mon + 1) * 100 +
                          now_tm.tm_mday;
        CJsonUniquePtr items(LoadScheduleItems());
        bool changed = false;

        for (int index = 0; index < cJSON_GetArraySize(items.get()); ++index) {
            cJSON* item = cJSON_GetArrayItem(items.get(), index);
            cJSON* scheduled_value = cJSON_GetObjectItem(item, "scheduled_at");
            cJSON* title_value = cJSON_GetObjectItem(item, "title");
            cJSON* repeat_value = cJSON_GetObjectItem(item, "repeat");
            if (!cJSON_IsNumber(scheduled_value) && cJSON_IsString(title_value)) {
                cJSON* time_value = cJSON_GetObjectItem(item, "time");
                time_t migrated_time;
                if (cJSON_IsString(time_value) &&
                    ParseScheduleTime(time_value->valuestring, migrated_time) &&
                    cJSON_AddStringToObject(item, "repeat", "once") != nullptr &&
                    cJSON_AddNumberToObject(item, "scheduled_at",
                                            static_cast<double>(migrated_time)) != nullptr &&
                    cJSON_AddBoolToObject(item, "notified", false) != nullptr &&
                    cJSON_AddNumberToObject(item, "last_fired_day", 0) != nullptr) {
                    changed = true;
                }
                continue;
            }
            if (!cJSON_IsNumber(scheduled_value) || !cJSON_IsString(title_value) ||
                !cJSON_IsString(repeat_value)) {
                continue;  // Keep legacy free-form entries, but they cannot trigger reminders.
            }

            const time_t scheduled_at = static_cast<time_t>(scheduled_value->valuedouble);
            if (now < scheduled_at) {
                continue;
            }

            bool due = false;
            if (std::strcmp(repeat_value->valuestring, "once") == 0) {
                cJSON* notified = cJSON_GetObjectItem(item, "notified");
                due = !cJSON_IsTrue(notified);
                if (due) {
                    cJSON_DeleteItemFromObject(item, "notified");
                    cJSON_AddBoolToObject(item, "notified", true);
                    changed = true;
                }
            } else {
                std::tm scheduled_tm = {};
                if (localtime_r(&scheduled_at, &scheduled_tm) == nullptr ||
                    (std::strcmp(repeat_value->valuestring, "weekly") == 0 &&
                     now_tm.tm_wday != scheduled_tm.tm_wday)) {
                    continue;
                }
                const int scheduled_minute = scheduled_tm.tm_hour * 60 + scheduled_tm.tm_min;
                const int current_minute = now_tm.tm_hour * 60 + now_tm.tm_min;
                cJSON* last_fired = cJSON_GetObjectItem(item, "last_fired_day");
                const int previous_day = cJSON_IsNumber(last_fired) ? last_fired->valueint : 0;
                due = current_minute >= scheduled_minute && previous_day != today;
                if (due) {
                    if (last_fired != nullptr) {
                        cJSON_SetNumberValue(last_fired, today);
                    } else {
                        cJSON_AddNumberToObject(item, "last_fired_day", today);
                    }
                    changed = true;
                }
            }

            if (due) {
                std::string notification = std::string("Đến giờ: ") + title_value->valuestring;
                Application::GetInstance().Schedule([notification = std::move(notification)]() {
                    auto& app = Application::GetInstance();
                    auto* display = Board::GetInstance().GetDisplay();
                    if (display != nullptr) {
                        display->ShowNotification(notification, 10000);
                    }
                    app.PlaySound(Lang::Sounds::OGG_EXCLAMATION);
                });
            }
        }

        if (changed) {
            SaveScheduleItems(items.get());
        }
    }

    static void ReminderMonitorTask(void* arg) {
        auto* board = static_cast<CompactWifiBoard*>(arg);
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(10000));
            board->CheckScheduleReminders();
        }
    }

    void InitializeTimeSync() {
        esp_sntp_config_t sntp_config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        esp_err_t result = esp_netif_sntp_init(&sntp_config);
        if (result != ESP_OK) {
            ESP_LOGW(TAG, "SNTP initialization failed: %s", esp_err_to_name(result));
        } else {
            time_sync_started_ = true;
        }
    }

    void InitializeScheduleReminders() {
        setenv("TZ", "ICT-7", 1);  // Asia/Bangkok (UTC+7, no daylight saving time).
        tzset();

        BaseType_t result_task = xTaskCreate(ReminderMonitorTask, "schedule_reminders", 4096,
                                             this, 2, nullptr);
        if (result_task != pdPASS) {
            ESP_LOGE(TAG, "Failed to start schedule reminder task");
        }
    }

    void InitializeTools() {
        static LampController lamp(LAMP_GPIO);

        auto& mcp_server = McpServer::GetInstance();
        mcp_server.AddTool(
            "self.schedule.add",
            "Remember and remind about an item at local Bangkok time. Format: YYYY-MM-DD HH:MM. "
            "repeat: once, daily, or weekly.",
            PropertyList({Property("title", kPropertyTypeString).SetMaxLength(96),
                          Property("time", kPropertyTypeString).SetMaxLength(16),
                          Property("repeat", kPropertyTypeString, "once")}),
            [this](const PropertyList& properties) -> ReturnValue {
                return AddScheduleItem(properties["title"].value<std::string>(),
                                       properties["time"].value<std::string>(),
                                       properties["repeat"].value<std::string>());
            });
        mcp_server.AddTool(
            "self.schedule.list", "List remembered schedule items and their IDs.", PropertyList(),
            [this](const PropertyList&) -> ReturnValue {
                std::lock_guard<std::mutex> lock(schedule_mutex_);
                CJsonUniquePtr items(LoadScheduleItems());
                CJsonStringUniquePtr json(cJSON_PrintUnformatted(items.get()));
                return json != nullptr ? ReturnValue(std::string(json.get()))
                                       : ReturnValue(std::string("[]"));
            });
        mcp_server.AddTool(
            "self.schedule.delete", "Delete a remembered schedule item by ID.",
            PropertyList({Property("id", kPropertyTypeInteger, 1, 2147483647)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return DeleteScheduleItem(properties["id"].value<int>());
            });
        mcp_server.AddTool(
            "self.display.set_emotion_mode",
            "Choose default emotion icon or animated blinking eyes. mode: default or animated; "
            "interval_ms controls the base blink interval.",
            PropertyList({Property("mode", kPropertyTypeString),
                          Property("interval_ms", kPropertyTypeInteger, 4000, 500, 30000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                auto mode = properties["mode"].value<std::string>();
                auto* display = static_cast<OledDisplay*>(GetDisplay());
                if (display == nullptr || !display->SetEmotionMode(
                                              mode,
                                              properties["interval_ms"].value<int>())) {
                    return std::string("Invalid mode; use default or animated");
                }
                return true;
            });

        mcp_server.AddTool(
            "self.chassis.move_forward",
            "Drive both wheels forward for duration_ms milliseconds (tiến thẳng trong thời gian yêu cầu).",
            PropertyList({Property("duration_ms", kPropertyTypeInteger, 1000, 100, 10000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return QueueMotorCommand(MotorAction::Forward,
                                         properties["duration_ms"].value<int>());
            });
        mcp_server.AddTool(
            "self.chassis.move_backward",
            "Drive both wheels backward for duration_ms milliseconds (lùi trong thời gian yêu cầu).",
            PropertyList({Property("duration_ms", kPropertyTypeInteger, 1000, 100, 10000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return QueueMotorCommand(MotorAction::Backward,
                                         properties["duration_ms"].value<int>());
            });
        mcp_server.AddTool(
            "self.chassis.turn_around",
            "Turn in place approximately 360 degrees (quay tại chỗ một vòng). Calibrate duration_ms.",
            PropertyList({Property("duration_ms", kPropertyTypeInteger,
                                   MOTOR_TURN_360_DURATION_MS, 100, 10000)}),
            [this](const PropertyList& properties) -> ReturnValue {
                return QueueMotorCommand(MotorAction::TurnAround,
                                         properties["duration_ms"].value<int>());
            });
        mcp_server.AddTool("self.chassis.stop", "Stop both wheels immediately (dừng bánh xe).", PropertyList(),
                           [this](const PropertyList&) -> ReturnValue {
                               return QueueMotorCommand(MotorAction::Stop, 0);
                           });
    }

    bool QueueMotorCommand(MotorAction action, int duration_ms) {
        if (motor_command_queue_ == nullptr) {
            return false;
        }

        MotorCommand command = {action, static_cast<uint32_t>(duration_ms)};
        return xQueueOverwrite(motor_command_queue_, &command) == pdPASS;
    }

    static void SetMotorInputs(int m1_in1, int m1_in2, int m2_in1, int m2_in2) {
        gpio_set_level(MOTOR_M1_IN1_GPIO, m1_in1);
        gpio_set_level(MOTOR_M1_IN2_GPIO, m1_in2);
        gpio_set_level(MOTOR_M2_IN1_GPIO, m2_in1);
        gpio_set_level(MOTOR_M2_IN2_GPIO, m2_in2);
    }

    static void MotorControlTask(void* arg) {
        auto* board = static_cast<CompactWifiBoard*>(arg);
        MotorAction current_action = MotorAction::Stop;
        TickType_t deadline = 0;

        while (true) {
            MotorCommand command;
            TickType_t wait_ticks = portMAX_DELAY;
            if (current_action != MotorAction::Stop) {
                TickType_t remaining = deadline - xTaskGetTickCount();
                wait_ticks = (remaining < (portMAX_DELAY / 2)) ? remaining : 0;
            }

            if (xQueueReceive(board->motor_command_queue_, &command, wait_ticks) == pdPASS) {
                current_action = command.action;
                switch (current_action) {
                    case MotorAction::Forward:
                        SetMotorInputs(1, 0, 1, 0);
                        break;
                    case MotorAction::Backward:
                        SetMotorInputs(0, 1, 0, 1);
                        break;
                    case MotorAction::TurnAround:
                        SetMotorInputs(1, 0, 0, 1);
                        break;
                    case MotorAction::Stop:
                    default:
                        SetMotorInputs(0, 0, 0, 0);
                        break;
                }
                deadline = xTaskGetTickCount() + pdMS_TO_TICKS(command.duration_ms);
            } else if (current_action != MotorAction::Stop) {
                current_action = MotorAction::Stop;
                SetMotorInputs(0, 0, 0, 0);
            }
        }
    }

    void InitializeMotors() {
        gpio_config_t motor_gpio_config = {
            .pin_bit_mask = (1ULL << MOTOR_M1_IN1_GPIO) | (1ULL << MOTOR_M1_IN2_GPIO) |
                            (1ULL << MOTOR_M2_IN1_GPIO) | (1ULL << MOTOR_M2_IN2_GPIO),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&motor_gpio_config));
        SetMotorInputs(0, 0, 0, 0);

        motor_command_queue_ = xQueueCreate(1, sizeof(MotorCommand));
        if (motor_command_queue_ == nullptr) {
            ESP_LOGE(TAG, "Failed to create motor command queue");
            return;
        }

        BaseType_t result =
            xTaskCreate(MotorControlTask, "motor_control", 2048, this, 3, nullptr);
        if (result != pdPASS) {
            ESP_LOGE(TAG, "Failed to start motor control task");
        }
    }

public:
    void SetNetworkEventCallback(NetworkEventCallback callback) override {
        WifiBoard::SetNetworkEventCallback(
            [this, callback = std::move(callback)](NetworkEvent event,
                                                   const std::string& data) {
                if (event == NetworkEvent::Connected && !time_sync_started_) {
                    InitializeTimeSync();
                }
                if (callback) {
                    callback(event, data);
                }
            });
    }

    CompactWifiBoard() :
        boot_button_(BOOT_BUTTON_GPIO),
        touch_button_(TOUCH_BUTTON_GPIO),
        volume_up_button_(VOLUME_UP_BUTTON_GPIO),
        volume_down_button_(VOLUME_DOWN_BUTTON_GPIO) {
        InitializeDisplayI2c();
        InitializeSsd1306Display();
        InitializeButtons();
        InitializeTools();
        InitializeMotors();
        InitializeScheduleReminders();
    }

    virtual Led* GetLed() override {
        static SingleLed led(BUILTIN_LED_GPIO);
        return &led;
    }

    virtual AudioCodec* GetAudioCodec() override {
#ifdef AUDIO_I2S_METHOD_SIMPLEX
        static NoAudioCodecSimplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_SPK_GPIO_BCLK, AUDIO_I2S_SPK_GPIO_LRCK, AUDIO_I2S_SPK_GPIO_DOUT, AUDIO_I2S_MIC_GPIO_SCK, AUDIO_I2S_MIC_GPIO_WS, AUDIO_I2S_MIC_GPIO_DIN);
#else
        static NoAudioCodecDuplex audio_codec(AUDIO_INPUT_SAMPLE_RATE, AUDIO_OUTPUT_SAMPLE_RATE,
            AUDIO_I2S_GPIO_BCLK, AUDIO_I2S_GPIO_WS, AUDIO_I2S_GPIO_DOUT, AUDIO_I2S_GPIO_DIN);
#endif
        return &audio_codec;
    }

    virtual Display* GetDisplay() override {
        return display_;
    }
};

DECLARE_BOARD(CompactWifiBoard);
