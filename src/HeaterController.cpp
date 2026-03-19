#include "HeaterController.h"
#include <PID_v1.h>
#include "ui.h"

HeaterController::HeaterController(uint8_t sensorPin, uint8_t heaterPin, const String& controlMode)
    : oneWire(sensorPin),
      sensors(&oneWire),
      heaterPin(heaterPin),
      controlMode(controlMode),
      controlActive(false),
      targetTemperature(25),  // Целое число по умолчанию
      currentTemperature(0.0),
      pwmChannel(0),
      controlTaskHandle(NULL),
      pidController(nullptr),
      pidKp(2.0), pidKi(5.0), pidKd(1.0) {
}

HeaterController::~HeaterController() {
    if (controlTaskHandle) {
        vTaskDelete(controlTaskHandle);
    }
    if (pidController) {
        delete pidController;
    }
}

void HeaterController::begin() {
    
    // Инициализация аппаратной части
    setupHardware();
    
    // Загрузка настроек из памяти
    preferences.begin("heater-ctrl", false);
    targetTemperature = preferences.getInt("targetTemp", 25);  // getInt вместо getFloat
    controlActive = preferences.getBool("controlActive", false);
    pidKp = preferences.getDouble("pidKp", 2.0);
    pidKi = preferences.getDouble("pidKi", 5.0);
    pidKd = preferences.getDouble("pidKd", 1.0);
    preferences.end();
    
    // Синхронизация с UI
    syncWithUI();
    
    // Создание задачи контроля температуры
    xTaskCreate(controlTask, "HeaterControl", 4096, this, 1, &controlTaskHandle);
}

void HeaterController::setupHardware() {
    // Инициализация датчика температуры
    sensors.begin();
    if (!sensors.getAddress(sensorAddress, 0)) {
        Serial.println("Temperature sensor not found!");
    }
    
    // Настройка вывода управления в зависимости от режима
    if (controlMode == "pid") {
        // Настройка ШИМ для ESP32
        pwmChannel = 0;
        ledcSetup(pwmChannel, 25000, 8);
        ledcAttachPin(heaterPin, pwmChannel);
        
        // Инициализация PID контроллера
        pidController = new PID(&pidInput, &pidOutput, &pidSetpoint, pidKp, pidKi, pidKd, DIRECT);
        pidController->SetMode(AUTOMATIC);
        pidController->SetOutputLimits(0, 255);
        pidSetpoint = (double)targetTemperature;  // Приведение к double для PID
    } else {
        // Релейный режим
        pinMode(heaterPin, OUTPUT);
        digitalWrite(heaterPin, LOW);
    }
}

void HeaterController::saveSettings() {
    preferences.begin("heater-ctrl", false);
    
    // Сохраняем все значения
    preferences.putInt("targetTemp", targetTemperature);
    preferences.putBool("controlActive", controlActive);
    preferences.putDouble("pidKp", pidKp);
    preferences.putDouble("pidKi", pidKi);
    preferences.putDouble("pidKd", pidKd);
    
    // Верифицируем только targetTemp и controlActive
    bool verificationOK = true;
    
    if (preferences.getInt("targetTemp", -999) != targetTemperature) {
        Serial.println("Verification failed: targetTemp");
        verificationOK = false;
    }
    if (preferences.getBool("controlActive", !controlActive) != controlActive) {
        Serial.println("Verification failed: controlActive");
        verificationOK = false;
    }
    
    preferences.end();
}

void HeaterController::syncWithUI() {
Serial.printf("Загружены настройки: Температура=%d°C, Контроль=%s, PID коэффициенты: P=%.1f, I=%.1f, D=%.1f\n",
                  targetTemperature, 
                  controlActive ? "ВКЛ" : "ВЫКЛ", 
                  pidKp, pidKi, pidKd);
    xSemaphoreTake(gui_mutex, portMAX_DELAY);
    
    // Синхронизация переключателя включения
    if (controlActive) {
        lv_obj_add_state(ui_SwitchHeaterOnOff, LV_STATE_CHECKED);
        Serial.println("SWITCH ON");
    } else {
        lv_obj_remove_state(ui_SwitchHeaterOnOff, LV_STATE_CHECKED);
        Serial.println("SWITCH OFF");
    }
    
    // Синхронизация целевой температуры
    lv_arc_set_value(ui_ArcHeatLevel, targetTemperature);
    Serial.println("ARC SET DONE");
    
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%d", targetTemperature);  // %d для целых чисел
    Serial.println("buffer target temp");
    Serial.print(buffer);
    lv_label_set_text(ui_LabelTempSetpoint, buffer);
    
    xSemaphoreGive(gui_mutex);
}

void HeaterController::controlTask(void* params) {
    HeaterController* controller = static_cast<HeaterController*>(params);
    
    while (1) {
        controller->updateTemperature();
        controller->controlHeater();
        vTaskDelay(pdMS_TO_TICKS(1000)); // Обновление каждую секунду
    }
}

void HeaterController::updateTemperature() {
    sensors.requestTemperatures();
    float temp = sensors.getTempC(sensorAddress);
    Serial.println(temp);
    
    if (temp != DEVICE_DISCONNECTED_C) {
        currentTemperature = temp;

        char buffer[20];
        int tempInt = (int)roundf(currentTemperature);
        snprintf(buffer, sizeof(buffer), "%d", tempInt);

        xSemaphoreTake(gui_mutex, portMAX_DELAY);

        // Экран 1
        if (lv_obj_is_visible(ui_LabelTempCurrent) &&
            lv_obj_get_screen(ui_LabelTempCurrent) == lv_scr_act()) {
            lv_label_set_text(ui_LabelTempCurrent, buffer);
        }

        // Экран 2
        if (lv_obj_is_visible(ui_heatpadTempLabel) &&
            lv_obj_get_screen(ui_heatpadTempLabel) == lv_scr_act()) {
            lv_label_set_text(ui_heatpadTempLabel, buffer);
        }

        xSemaphoreGive(gui_mutex);
    }
}

void HeaterController::controlHeater() {
    if (!controlActive) {
        // Если контроль выключен - выключаем обогреватель
        if (controlMode == "pid") {
            ledcWrite(pwmChannel, 0);
        } else {
            digitalWrite(heaterPin, LOW);
        }
        return;
    }
    
    if (controlMode == "relay") {
        // Релейное управление с гистерезисом
        if (currentTemperature < targetTemperature - 0.5) {
            digitalWrite(heaterPin, HIGH);
        } else if (currentTemperature > targetTemperature + 0.5) {
            digitalWrite(heaterPin, LOW);
        }
    } else if (controlMode == "pid") {
        // PID управление
        pidInput = currentTemperature;
        pidSetpoint = (double)targetTemperature;  // Приведение к double
        pidController->Compute();
        ledcWrite(pwmChannel, (uint32_t)pidOutput);
    }
}

void HeaterController::setEnabled(bool enabled) {
    controlActive = enabled;
    saveSettings();
}

void HeaterController::setTargetTemperature(int temperature) {  // Параметр изменен на int
    targetTemperature = temperature;
    saveSettings();
}