#include "HeaterController.h"

HeaterController::HeaterController(int sensorPin, int heaterPin, const String& controlMode)
    : heaterPin(heaterPin), 
      controlMode(controlMode),
      targetTemp(0.0),
      currentTemp(0.0),
      pwmChannel(0) {
    
    // Инициализация семафоров
    tempMutex = xSemaphoreCreateMutex();
    pidMutex = xSemaphoreCreateMutex();
    
    // Инициализация датчика температуры
    oneWire = new OneWire(sensorPin);
    sensors = new DallasTemperature(oneWire);
    sensors->begin();
    
    if (!sensors->getAddress(sensorAddress, 0)) {
        Serial.println("Temperature sensor not found!");
    }
    
    // Настройка вывода управления
    if (controlMode == "pid") {
        ledcSetup(pwmChannel, 5000, 8);
        ledcAttachPin(heaterPin, pwmChannel);
        pidController = new PID(&pidInput, &pidOutput, &pidSetpoint, 2.0, 5.0, 1.0, DIRECT);
        pidController->SetMode(AUTOMATIC);
        pidController->SetOutputLimits(0, 255);
    } else {
        pinMode(heaterPin, OUTPUT);
        digitalWrite(heaterPin, LOW);
    }
    
    // Создание задач FreeRTOS
    xTaskCreatePinnedToCore(
        temperatureTaskWrapper,
        "TempTask",
        4096,
        this,
        2,
        &tempTaskHandle,
        0
    );
    
    xTaskCreatePinnedToCore(
        controlTaskWrapper,
        "ControlTask",
        4096,
        this,
        3,
        &controlTaskHandle,
        0
    );
}

HeaterController::~HeaterController() {
    if (tempTaskHandle) vTaskDelete(tempTaskHandle);
    if (controlTaskHandle) vTaskDelete(controlTaskHandle);
    vSemaphoreDelete(tempMutex);
    vSemaphoreDelete(pidMutex);
    
    delete sensors;
    delete oneWire;
    if (controlMode == "pid") delete pidController;
}

void HeaterController::setTargetTemperature(float temp) {
    xSemaphoreTake(tempMutex, portMAX_DELAY);
    targetTemp = temp;
    if (controlMode == "pid") {
        xSemaphoreTake(pidMutex, portMAX_DELAY);
        pidSetpoint = temp;
        xSemaphoreGive(pidMutex);
    }
    xSemaphoreGive(tempMutex);
}

void HeaterController::setPIDTunings(double Kp, double Ki, double Kd) {
    if (controlMode == "pid") {
        xSemaphoreTake(pidMutex, portMAX_DELAY);
        pidController->SetTunings(Kp, Ki, Kd);
        xSemaphoreGive(pidMutex);
    }
}

// Статические методы-обертки
void HeaterController::temperatureTaskWrapper(void* params) {
    HeaterController* instance = static_cast<HeaterController*>(params);
    instance->temperatureTask();
}

void HeaterController::controlTaskWrapper(void* params) {
    HeaterController* instance = static_cast<HeaterController*>(params);
    instance->controlTask();
}

// Реализация задач
void HeaterController::temperatureTask() {
    const TickType_t xFrequency = pdMS_TO_TICKS(1000);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        sensors->requestTemperatures();
        float temp = sensors->getTempC(sensorAddress);
        
        if (temp != DEVICE_DISCONNECTED_C) {
            xSemaphoreTake(tempMutex, portMAX_DELAY);
            currentTemp = temp;
            xSemaphoreGive(tempMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void HeaterController::controlTask() {
    const TickType_t xFrequency = pdMS_TO_TICKS(100);
    TickType_t xLastWakeTime = xTaskGetTickCount();
    
    while (true) {
        xSemaphoreTake(tempMutex, portMAX_DELAY);
        float localTarget = targetTemp;
        float localCurrent = currentTemp;
        xSemaphoreGive(tempMutex);
        
        if (controlMode == "relay") {
            digitalWrite(heaterPin, localCurrent < localTarget ? HIGH : LOW);
        }
        else if (controlMode == "pid") {
            xSemaphoreTake(pidMutex, portMAX_DELAY);
            pidInput = localCurrent;
            pidController->Compute();
            ledcWrite(pwmChannel, (uint32_t)pidOutput);
            xSemaphoreGive(pidMutex);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}