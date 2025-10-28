#ifndef HUMIDITY_CONTROLLER_H
#define HUMIDITY_CONTROLLER_H

#include <Arduino.h>
#include <AHTxx.h>
#include <Wire.h>
#include <ui.h>
#include <Preferences.h>
#include <driver/ledc.h>

class HumidityController {
private:
    AHTxx ahtSensor;
    Preferences preferences;
    uint8_t pumpPwmPin;
    
    uint8_t targetHumidity;
    uint8_t pumpDuration;
    uint8_t pumpPower;
    bool controlActive;
    bool pumpRunning;

    ledc_channel_t pwmChannel;
    TaskHandle_t controlTaskHandle;

    void setupPWM();
    void activatePump();
    void deactivatePump();
    void saveSettings();
    static void controlTask(void* params);

public:
    HumidityController(uint8_t sdaPin, uint8_t sclPin, uint8_t pwmPin);
    void begin();
    void setTargetHumidity(uint8_t humidity);
    void setPumpDuration(uint8_t seconds);
    bool getSensorData(float &temperature, float &humidity);
    void enableControl(bool enable);
    void setPumpPower(uint8_t power);
};

#endif