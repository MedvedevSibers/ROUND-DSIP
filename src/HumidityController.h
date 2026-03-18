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
    uint8_t waterLevelPin;
    uint8_t uvSterilizationPin;
    
    uint8_t targetHumidity;
    uint8_t pumpDuration;
    uint8_t pumpPower;
    bool controlActive;
    bool pumpRunning;
    bool waterAvailable;
    
    // УФ-стерилизация
    bool uvSterilizationActive;
    bool uvSterilizationRunning;
    unsigned long uvInterval;        // Интервал между включениями (мс)
    unsigned long uvDuration;        // Длительность работы (мс)
    unsigned long lastUvActivation;  // Время последнего включения

    // Защита от частых включений помпы
    unsigned long lastPumpActivation; // Время последней активации помпы
    const unsigned long PUMP_COOLDOWN = 300000; // 5 минут в миллисекундах

    ledc_channel_t pwmChannel;
    TaskHandle_t controlTaskHandle;
    TaskHandle_t uvTaskHandle;

    void setupPWM();
    void activatePump();
    void deactivatePump();
    void activateUvSterilization();
    void deactivateUvSterilization();
    void saveSettings();
    void checkWaterLevel();
    static void controlTask(void* params);
    static void uvSterilizationTask(void* params);

public:
    HumidityController(uint8_t sdaPin, uint8_t sclPin, uint8_t pwmPin, uint8_t waterLevelPin, uint8_t uvPin);
    void begin();
    void setTargetHumidity(uint8_t humidity);
    void setPumpDuration(uint8_t seconds);
    bool getSensorData(float &temperature, float &humidity);
    void enableControl(bool enable);
    void setPumpPower(uint8_t power);
    bool isWaterAvailable();
    
    // УФ-стерилизация
    void enableUvSterilization(bool enable);
    void setUvInterval(unsigned long intervalMinutes);
    void setUvDuration(unsigned long durationMinutes);
    bool isUvSterilizationActive();
    bool isUvSterilizationRunning();
    unsigned long getUvInterval();
    unsigned long getUvDuration();
    
    // Для отладки
    unsigned long getTimeUntilNextPumpActivation();
};

#endif