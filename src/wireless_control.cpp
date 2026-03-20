#include "wireless_control.h"
#include <Arduino.h>
#include <cstring>

void AsyncWiFiManager::loadCredentials() {
    prefs.begin("wifi", true);
    strncpy(ssid, prefs.getString("ssid", "").c_str(), SSID_LENGTH);
    strncpy(password, prefs.getString("password", "").c_str(), PASSWORD_LENGTH);
    prefs.end();
    Serial.println("Loaded WiFi credentials from NVS");
}

void AsyncWiFiManager::saveCredentials() {
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("password", password);
    prefs.end();
    Serial.println("Saved WiFi credentials to NVS");
}

void AsyncWiFiManager::taskHandler(void* params) {
    AsyncWiFiManager* instance = static_cast<AsyncWiFiManager*>(params);
    instance->wifiTask();
}

void AsyncWiFiManager::wifiTask() {
    for (;;) {
        switch (WiFi.status()) {
            case WL_CONNECTED:
                xEventGroupSetBits(wifiEventGroup, WIFI_CONNECTED_BIT);
                connectionInProgress = false;
                vTaskDelay(pdMS_TO_TICKS(1000));
                break;

            case WL_NO_SSID_AVAIL:
            case WL_CONNECT_FAILED:
            case WL_DISCONNECTED:
                if (!connectionInProgress) {
                    connectionInProgress = true;
                    reconnect();
                }
                xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
                vTaskDelay(pdMS_TO_TICKS(5000));
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// ===== PUBLIC =====

AsyncWiFiManager::AsyncWiFiManager() : connectionInProgress(false) {
    wifiEventGroup = xEventGroupCreate();
    memset(ssid, 0, SSID_LENGTH);
    memset(password, 0, PASSWORD_LENGTH);
}

AsyncWiFiManager::~AsyncWiFiManager() {
    vEventGroupDelete(wifiEventGroup);
    if (wifiTaskHandle) vTaskDelete(wifiTaskHandle);
}

void AsyncWiFiManager::begin() {
    loadCredentials();

    if (strlen(ssid) > 0 && strlen(password) > 0) {
        WiFi.begin(ssid, password);
    }

    xTaskCreatePinnedToCore(
        taskHandler,
        "WiFiManagerTask",
        10000,
        this,
        1,
        &wifiTaskHandle,
        1
    );
}

void AsyncWiFiManager::connect(const char* newSSID, const char* newPassword) {
    strncpy(ssid, newSSID, SSID_LENGTH);
    strncpy(password, newPassword, PASSWORD_LENGTH);
    saveCredentials();

    WiFi.begin(ssid, password);
    connectionInProgress = true;
}

void AsyncWiFiManager::disconnect() {
    WiFi.disconnect(true);
    xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
    connectionInProgress = false;
}

void AsyncWiFiManager::reconnect() {
    if (strlen(ssid) > 0 && strlen(password) > 0) {
        WiFi.begin(ssid, password);
        Serial.println("reconnect");
        connectionInProgress = true;
    }
}

bool AsyncWiFiManager::isConnected() {
    return (xEventGroupGetBits(wifiEventGroup) & WIFI_CONNECTED_BIT) &&
           (WiFi.status() == WL_CONNECTED);
}

String AsyncWiFiManager::getIP() {
    return WiFi.localIP().toString();
}