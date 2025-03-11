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

    void loadCredentials() {
        prefs.begin("wifi", true);
        strncpy(ssid, prefs.getString("ssid", "").c_str(), SSID_LENGTH);
        strncpy(password, prefs.getString("password", "").c_str(), PASSWORD_LENGTH);
        prefs.end();
        Serial.println("Loaded WiFi credentials from NVS");
    }

    void saveCredentials() {
        prefs.begin("wifi", false);
        prefs.putString("ssid", ssid);
        prefs.putString("password", password);
        prefs.end();
        Serial.println("Saved WiFi credentials to NVS");
    }

    static void taskHandler(void* params) {
        AsyncWiFiManager* instance = static_cast<AsyncWiFiManager*>(params);
        instance->wifiTask();
    }

    void wifiTask() {
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

  public:
    AsyncWiFiManager() : connectionInProgress(false) {
        wifiEventGroup = xEventGroupCreate();
        memset(ssid, 0, SSID_LENGTH);
        memset(password, 0, PASSWORD_LENGTH);
    }

    ~AsyncWiFiManager() {
        vEventGroupDelete(wifiEventGroup);
        if (wifiTaskHandle) vTaskDelete(wifiTaskHandle);
    }

    void begin() {
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

    void connect(const char* newSSID, const char* newPassword) {
        strncpy(ssid, newSSID, SSID_LENGTH);
        strncpy(password, newPassword, PASSWORD_LENGTH);
        saveCredentials();
        WiFi.begin(ssid, password);
        connectionInProgress = true;
    }

    void disconnect() {
        WiFi.disconnect(true);
        xEventGroupClearBits(wifiEventGroup, WIFI_CONNECTED_BIT);
        connectionInProgress = false;
    }

    void reconnect() {
        if (strlen(ssid) > 0 && strlen(password) > 0) {
            WiFi.begin(ssid, password);
            Serial.print("reconnect");
            connectionInProgress = true;
        }
    }

    bool isConnected() {
        return (xEventGroupGetBits(wifiEventGroup) & WIFI_CONNECTED_BIT) &&
               (WiFi.status() == WL_CONNECTED);
    }

    String getIP() {
        return WiFi.localIP().toString();
    }
};