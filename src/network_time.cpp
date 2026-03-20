#include "network_time.h"
#include <WiFi.h>
#include <time.h>
#include "wireless_control.h"

// NTP настройки
static const char* NTP_SERVER = "pool.ntp.org";
static const long GMT_OFFSET_SEC = 7 * 3600; // +7
static const int DAYLIGHT_OFFSET_SEC = 0;

NetworkTime::NetworkTime(AsyncWiFiManager* wifi)
    : wifiManager(wifi), taskHandle(nullptr), timeInitialized(false) {}

NetworkTime::~NetworkTime() {
    if (taskHandle) {
        vTaskDelete(taskHandle);
    }
}

void NetworkTime::begin() {
    xTaskCreatePinnedToCore(
        taskFunc,
        "NetworkTimeTask",
        4096,
        this,
        1,
        &taskHandle,
        1
    );
}

void NetworkTime::taskFunc(void* param) {
    NetworkTime* instance = static_cast<NetworkTime*>(param);
    instance->task();
}

void NetworkTime::initTime() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);

    struct tm timeinfo;
    if (getLocalTime(&timeinfo, 10000)) {
        Serial.println("Time initialized via NTP");
        timeInitialized = true;
    } else {
        Serial.println("Failed to get time");
    }
}

void NetworkTime::updateLabel(const char* timeStr) {
    xSemaphoreTake(gui_mutex, portMAX_DELAY);

    if (ui_timeLabel &&
        lv_obj_is_visible(ui_timeLabel) &&
        lv_obj_get_screen(ui_timeLabel) == lv_scr_act()) {

        lv_label_set_text(ui_timeLabel, timeStr);
    }

    xSemaphoreGive(gui_mutex);
}

void NetworkTime::task() {
    char buffer[32];

    for (;;) {
        // Ждём WiFi
        if (wifiManager->isConnected()) {

            if (!timeInitialized) {
                initTime();
            }

            if (timeInitialized) {
                struct tm timeinfo;

                if (getLocalTime(&timeinfo)) {
                    strftime(buffer, sizeof(buffer), "%H:%M:%S", &timeinfo);
                    updateLabel(buffer);
                }
            }
        } else {
            timeInitialized = false; // сброс при потере сети
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}