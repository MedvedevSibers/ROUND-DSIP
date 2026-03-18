#include "HumidityController.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

HumidityController::HumidityController(uint8_t sdaPin, uint8_t sclPin, uint8_t pwmPin, uint8_t waterLevelPin, uint8_t uvPin)
    : ahtSensor(AHTXX_ADDRESS_X38, AHT1x_SENSOR),
      pumpPwmPin(pwmPin),
      waterLevelPin(waterLevelPin),
      uvSterilizationPin(uvPin),
      pwmChannel(LEDC_CHANNEL_0),
      controlTaskHandle(NULL),
      uvTaskHandle(NULL),
      uvSterilizationActive(false),
      uvSterilizationRunning(false),
      uvInterval(3600000),    // 1 час по умолчанию
      uvDuration(60000),      // 1 минута по умолчанию
      lastUvActivation(0),
      lastPumpActivation(0) {
    Wire.begin(sdaPin, sclPin);
}

void HumidityController::begin() {
    ahtSensor.begin();
    setupPWM();
    
    // Инициализация пинов
    pinMode(waterLevelPin, INPUT_PULLUP); // Для опторазвязки
    pinMode(uvSterilizationPin, OUTPUT);
    digitalWrite(uvSterilizationPin, LOW); // Выключаем УФ-лампу при старте

    preferences.begin("humidity-ctrl", false);
    targetHumidity = preferences.getInt("targetHumid", 50);
    pumpDuration = preferences.getUChar("pumpDuration", 5);
    pumpPower = preferences.getUChar("pumpPower", 200);
    controlActive = preferences.getBool("controlActive", false);
    
    // Загрузка настроек УФ-стерилизации
    uvSterilizationActive = preferences.getBool("uvActive", false);
    uvInterval = preferences.getULong("uvInterval", 3600000);  // 1 час
    uvDuration = preferences.getULong("uvDuration", 60000);    // 1 минута
    
    preferences.end();
    
    char buffer[20];
    snprintf(buffer, sizeof(buffer), "%d", targetHumidity);
    xSemaphoreTake(gui_mutex, portMAX_DELAY);                 
    if (controlActive) {
        lv_obj_add_state(ui_SwitchMoistOnOff,LV_STATE_CHECKED);
    }
    else {
        lv_obj_remove_state(ui_SwitchMoistOnOff,LV_STATE_CHECKED);
    }
    lv_label_set_text(ui_LableMoistureTraget,buffer);
    lv_arc_set_value(ui_ArcMoistLevel,targetHumidity);
    lv_slider_set_value(ui_SliderMoistureVolume,pumpDuration,LV_ANIM_OFF);
    lv_slider_set_value(ui_SliderPumpPower,pumpPower,LV_ANIM_OFF);
    xSemaphoreGive(gui_mutex);

    xTaskCreate(controlTask, "HumidityControl", 4096, this, 1, &controlTaskHandle);
    xTaskCreate(uvSterilizationTask, "UvSterilization", 4096, this, 1, &uvTaskHandle);
}

void HumidityController::setupPWM() {
    ledc_timer_config_t timerConf = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 20000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timerConf);

    ledc_channel_config_t channelConf = {
        .gpio_num = pumpPwmPin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = pwmChannel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0
    };
    ledc_channel_config(&channelConf);
}

void HumidityController::checkWaterLevel() {
    // Для опторазвязки: LOW = вода есть, HIGH = воды нет
    bool waterDetected = (digitalRead(waterLevelPin) == LOW);
    
    if (waterDetected != waterAvailable) {
        waterAvailable = waterDetected;
        
        // TODO: Раскомментировать когда будет реализован ui_waterAlert
        /*
        xSemaphoreTake(gui_mutex, portMAX_DELAY);
        if (waterAvailable) {
            lv_obj_add_flag(ui_waterAlert, LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_clear_flag(ui_waterAlert, LV_OBJ_FLAG_HIDDEN);
        }
        xSemaphoreGive(gui_mutex);
        */
        
        Serial.printf("Water level changed: %s\n", waterAvailable ? "AVAILABLE" : "LOW");
        
        // Если вода закончилась, выключаем УФ-стерилизацию
        if (!waterAvailable && uvSterilizationRunning) {
            deactivateUvSterilization();
        }
    }
}

void HumidityController::controlTask(void* params) {
    HumidityController* controller = static_cast<HumidityController*>(params);
    unsigned long lastWaterCheck = 0;
    const unsigned long WATER_CHECK_INTERVAL = 5000; // 5 секунд
    
    while(1) {
        unsigned long currentTime = millis();
        
        // Проверяем уровень воды каждые 5 секунд
        if (currentTime - lastWaterCheck >= WATER_CHECK_INTERVAL) {
            controller->checkWaterLevel();
            lastWaterCheck = currentTime;
        }
        
        if(controller->controlActive && controller->waterAvailable) {
            float humidity = controller->ahtSensor.readHumidity();
            float temperature = controller->ahtSensor.readTemperature();
            bool container_visible = lv_obj_is_visible(ui_MoistureContainer);
            if (container_visible) {
                char buffer[20];
                int humInt = (int)roundf(humidity);
                snprintf(buffer, sizeof(buffer), "%d", humInt);
                xSemaphoreTake(gui_mutex, portMAX_DELAY);
                lv_label_set_text(ui_LableMoistureCurrent, buffer);
                xSemaphoreGive(gui_mutex);
            }
            
            if(!isnan(humidity) && !isnan(temperature)) {
                // Проверяем, прошло ли достаточно времени с последней активации помпы
                bool pumpCooldownElapsed = (currentTime - controller->lastPumpActivation >= controller->PUMP_COOLDOWN);
                
                if(humidity < controller->targetHumidity && 
                   !controller->pumpRunning && 
                   pumpCooldownElapsed) {
                    controller->activatePump();
                } else if (humidity < controller->targetHumidity && 
                          !controller->pumpRunning && 
                          !pumpCooldownElapsed) {
                    // Логируем, что помпа не активируется из-за cooldown
                    unsigned long remainingTime = controller->PUMP_COOLDOWN - (currentTime - controller->lastPumpActivation);
                    Serial.printf("Pump activation delayed: %lu seconds remaining\n", remainingTime / 1000);
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void HumidityController::uvSterilizationTask(void* params) {
    HumidityController* controller = static_cast<HumidityController*>(params);
    
    while(1) {
        if (controller->uvSterilizationActive && controller->waterAvailable) {
            unsigned long currentTime = millis();
            
            // Проверяем, пора ли включить УФ-стерилизацию
            if (!controller->uvSterilizationRunning && 
                (currentTime - controller->lastUvActivation >= controller->uvInterval)) {
                controller->activateUvSterilization();
            }
            
            // Проверяем, пора ли выключить УФ-стерилизацию
            if (controller->uvSterilizationRunning && 
                (currentTime - controller->lastUvActivation >= controller->uvDuration)) {
                controller->deactivateUvSterilization();
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000)); // Проверяем каждую секунду
    }
}

void HumidityController::activatePump() {
    // Дополнительная проверка наличия воды перед запуском
    if (!waterAvailable) {
        Serial.println("Pump activation blocked: no water available");
        return;
    }
    
    pumpRunning = true;
    lastPumpActivation = millis(); // Запоминаем время активации
    
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, pumpPower);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    
    Serial.println("Pump activated");
    
    xTaskCreate(
        [](void* params) {
            HumidityController* ctrl = static_cast<HumidityController*>(params);
            vTaskDelay(pdMS_TO_TICKS(ctrl->pumpDuration * 1000));
            ctrl->deactivatePump();
            vTaskDelete(NULL);
        },
        "PumpControl",
        2048,
        this,
        1,
        NULL
    );
}

void HumidityController::deactivatePump() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    pumpRunning = false;
    Serial.println("Pump deactivated");
}

void HumidityController::activateUvSterilization() {
    if (!waterAvailable) {
        Serial.println("UV sterilization blocked: no water available");
        return;
    }
    
    uvSterilizationRunning = true;
    lastUvActivation = millis();
    digitalWrite(uvSterilizationPin, HIGH);
    
    Serial.println("UV sterilization activated");
}

void HumidityController::deactivateUvSterilization() {
    uvSterilizationRunning = false;
    lastUvActivation = millis();
    digitalWrite(uvSterilizationPin, LOW);
    
    Serial.println("UV sterilization deactivated");
}

void HumidityController::saveSettings() {
    preferences.begin("humidity-ctrl", false);
    preferences.putInt("targetHumid", targetHumidity);
    preferences.putUChar("pumpDuration", pumpDuration);
    preferences.putUChar("pumpPower", pumpPower);
    preferences.putBool("controlActive", controlActive);
    
    // Сохранение настроек УФ-стерилизации
    preferences.putBool("uvActive", uvSterilizationActive);
    preferences.putULong("uvInterval", uvInterval);
    preferences.putULong("uvDuration", uvDuration);
    
    preferences.end();
}

// Методы УФ-стерилизации
void HumidityController::enableUvSterilization(bool enable) {
    uvSterilizationActive = enable;
    
    // Если выключаем стерилизацию, то выключаем и лампу
    if (!enable && uvSterilizationRunning) {
        deactivateUvSterilization();
    }
    
    saveSettings();
}

void HumidityController::setUvInterval(unsigned long intervalMinutes) {
    uvInterval = intervalMinutes * 60000; // Конвертируем минуты в миллисекунды
    saveSettings();
}

void HumidityController::setUvDuration(unsigned long durationMinutes) {
    uvDuration = durationMinutes * 60000; // Конвертируем минуты в миллисекунды
    saveSettings();
}

bool HumidityController::isUvSterilizationActive() {
    return uvSterilizationActive;
}

bool HumidityController::isUvSterilizationRunning() {
    return uvSterilizationRunning;
}

unsigned long HumidityController::getUvInterval() {
    return uvInterval / 60000; // Возвращаем в минутах
}

unsigned long HumidityController::getUvDuration() {
    return uvDuration / 60000; // Возвращаем в минутах
}

// Для отладки - получение оставшегося времени до следующей возможной активации помпы
unsigned long HumidityController::getTimeUntilNextPumpActivation() {
    unsigned long currentTime = millis();
    unsigned long elapsed = currentTime - lastPumpActivation;
    
    if (elapsed >= PUMP_COOLDOWN) {
        return 0;
    } else {
        return PUMP_COOLDOWN - elapsed;
    }
}

// Существующие методы
void HumidityController::setTargetHumidity(uint8_t humidity) {
    if(humidity >= 0 && humidity <= 100) {
        targetHumidity = humidity;
        saveSettings();
    }
}

void HumidityController::setPumpDuration(uint8_t seconds) {
    if(seconds >= 1 && seconds <= 10) {
        pumpDuration = seconds;
        saveSettings();
    }
}

bool HumidityController::getSensorData(float &temperature, float &humidity) {
    humidity = ahtSensor.readHumidity();
    temperature = ahtSensor.readTemperature();
    return !isnan(humidity) && !isnan(temperature);
}

void HumidityController::enableControl(bool enable) {
    controlActive = enable;
    saveSettings();
}

void HumidityController::setPumpPower(uint8_t power) {
    pumpPower = constrain(power, 0, 255);
    saveSettings();
}

bool HumidityController::isWaterAvailable() {
    return waterAvailable;
}