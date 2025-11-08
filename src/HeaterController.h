#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

class HeaterController {
public:
    HeaterController(uint8_t sensorPin, uint8_t heaterPin, const String& controlMode);
    ~HeaterController();
    
    void begin();
    void saveSettings();
    
    // Управление состоянием
    void setEnabled(bool enabled);
    void setTargetTemperature(int temperature);  // Изменено на int
    
    // Геттеры
    bool isEnabled() const { return controlActive; }
    int getTargetTemperature() const { return targetTemperature; }  // Изменено на int
    float getCurrentTemperature() const { return currentTemperature; }
    
    // UI синхронизация
    void syncWithUI();

private:
    // Аппаратные компоненты
    OneWire oneWire;
    DallasTemperature sensors;
    DeviceAddress sensorAddress;
    uint8_t heaterPin;
    String controlMode;
    
    // Состояние системы
    volatile bool controlActive;
    volatile int targetTemperature;  // Изменено на int
    volatile float currentTemperature;
    
    // PID компоненты
    class PID* pidController;
    double pidInput, pidOutput, pidSetpoint;
    uint8_t pwmChannel;
    
    // RTOS компоненты
    TaskHandle_t controlTaskHandle;
    Preferences preferences;
    
    // Приватные методы
    void setupHardware();
    static void controlTask(void* params);
    void updateTemperature();
    void controlHeater();
    
    // PID настройки
    double pidKp, pidKi, pidKd;
};

#endif