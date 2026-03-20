#pragma once

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
class AsyncWiFiManager;

// Внешние зависимости
extern SemaphoreHandle_t gui_mutex;

// LVGL label (объявлен где-то в ui)
extern lv_obj_t* ui_timeLabel;

class AsyncWiFiManager; // forward declaration

class NetworkTime {
public:
    NetworkTime(AsyncWiFiManager* wifi);
    ~NetworkTime();

    void begin();

private:
    AsyncWiFiManager* wifiManager;
    TaskHandle_t taskHandle;

    static void taskFunc(void* param);
    void task();

    void initTime();
    void updateLabel(const char* timeStr);

    bool timeInitialized;
};