#ifndef HEATER_CONTROLLER_H
#define HEATER_CONTROLLER_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PID_v1.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "ui.h"


class HeaterController {
public:
    HeaterController(int sensorPin, int heaterPin, const String& controlMode);
    ~HeaterController();
    
    void setTargetTemperature(float temp);
    void setPIDTunings(double Kp, double Ki, double Kd);

private:
    // Конфигурационные параметры
    int heaterPin;
    String controlMode;
    
    // Состояние системы
    volatile float targetTemp;
    volatile float currentTemp;
    
    // Компоненты датчика температуры
    OneWire* oneWire;
    DallasTemperature* sensors;
    DeviceAddress sensorAddress;
    
    // PID компоненты
    PID* pidController;
    double pidInput, pidOutput, pidSetpoint;
    int pwmChannel;
    
    // RTOS компоненты
    TaskHandle_t tempTaskHandle;
    TaskHandle_t controlTaskHandle;
    SemaphoreHandle_t tempMutex;
    SemaphoreHandle_t pidMutex;

    // Приватные методы
    void temperatureTask();
    void controlTask();
    
    // Статические обертки для задач FreeRTOS
    static void temperatureTaskWrapper(void* params);
    static void controlTaskWrapper(void* params);
};

#endif