#include "HumidityController.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

HumidityController::HumidityController(uint8_t sdaPin, uint8_t sclPin, uint8_t enablePin, uint8_t pwmPin)
    : ahtSensor(AHTXX_ADDRESS_X38, AHT1x_SENSOR),
      pumpEnablePin(enablePin),
      pumpPwmPin(pwmPin),
      pwmChannel(LEDC_CHANNEL_0),
      controlTaskHandle(NULL) {
    Wire.begin(sdaPin, sclPin);
}

void HumidityController::begin() {
    ahtSensor.begin();
    pinMode(pumpEnablePin, OUTPUT);
    digitalWrite(pumpEnablePin, LOW);
    setupPWM();

    preferences.begin("humidity-ctrl", false);
    targetHumidity = preferences.getFloat("targetHumid", 50.0f);
    pumpDuration = preferences.getUChar("pumpDuration", 5);
    pumpPower = preferences.getUChar("pumpPower", 200);
    controlActive = preferences.getBool("controlActive", false);
    preferences.end();

    xTaskCreate(controlTask, "HumidityControl", 4096, this, 1, &controlTaskHandle);
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

void HumidityController::saveSettings() {
    preferences.begin("humidity-ctrl", false);
    preferences.putFloat("targetHumid", targetHumidity);
    preferences.putUChar("pumpDuration", pumpDuration);
    preferences.putUChar("pumpPower", pumpPower);
    preferences.putBool("controlActive", controlActive);
    preferences.end();
}

void HumidityController::controlTask(void* params) {
    HumidityController* controller = static_cast<HumidityController*>(params);
    while(1) {
        if(controller->controlActive) {
            float humidity = controller->ahtSensor.readHumidity();
            float temperature = controller->ahtSensor.readTemperature();
            
            if(!isnan(humidity) && !isnan(temperature)) {
                if(humidity < controller->targetHumidity && !controller->pumpRunning) {
                    controller->activatePump();
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void HumidityController::activatePump() {
    pumpRunning = true;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, pumpPower);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    digitalWrite(pumpEnablePin, HIGH);
    
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
    digitalWrite(pumpEnablePin, LOW);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, pwmChannel, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, pwmChannel);
    pumpRunning = false;
}

void HumidityController::setTargetHumidity(float humidity) {
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