#pragma once

#include <WiFi.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>

#define SSID_LENGTH 32
#define PASSWORD_LENGTH 64
#define WIFI_CONNECTED_BIT BIT0

class AsyncWiFiManager {
  private:
    Preferences prefs;
    EventGroupHandle_t wifiEventGroup;
    TaskHandle_t wifiTaskHandle;
    char ssid[SSID_LENGTH];
    char password[PASSWORD_LENGTH];
    bool connectionInProgress;

    void loadCredentials();
    void saveCredentials();

    static void taskHandler(void* params);
    void wifiTask();

  public:
    AsyncWiFiManager();
    ~AsyncWiFiManager();

    void begin();
    void connect(const char* newSSID, const char* newPassword);
    void disconnect();
    void reconnect();

    bool isConnected();
    String getIP();
};