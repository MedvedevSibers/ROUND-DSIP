#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <CST816S.h>
#include <QuickPID.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include "HeaterController.h"
#include "wireless_control.cpp"
#include "HumidityController.h"

#define RW_MODE false  //Варианты работы с памятью
#define RO_MODE true

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

AsyncWiFiManager wifiManager;

HeaterController* heater;

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */
CST816S mytouch(22,21,27,14); // пины для работы с тачскрином 22 sda 21 scl

#define PUMP_PIN 13
#define TEMP_PIN 32
#define RELAY_PIN 33 // WAS 33
#define SDA_PIN 22
#define SCL_PIN 21

HumidityController humidityControl(SDA_PIN,SCL_PIN,PUMP_PIN);


OneWire oneWire(TEMP_PIN);
DallasTemperature temp(&oneWire);

Preferences nvs;

float floor_temp = 32; // текущая температура
int floor_setpoint; // установленная температура пола
float saved_temp; // переменная для загрузки-выгрузки сохраненной температуры


#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

static uint32_t my_tick_get_cb (void) { return millis(); }

void initPwmSetup() {
    ledcSetup(0,20000,8);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
}

void initNvs() {
  nvs.begin("stored_values", RW_MODE);
  bool tpInit = nvs.isKey("nvsInit");
  if (tpInit == false) {
    nvs.end();
    nvs.begin("stored_values", RW_MODE);
    nvs.putInt("saved_temp", 30);
    nvs.putBool("nvsInit", true);
    nvs.end();
    nvs.begin("stored_values", RO_MODE);
  }
  floor_setpoint = nvs.getInt("saved_temp");
  nvs.end();
}

static void event_arc_temp_change (lv_event_t * e) {
  int val = lv_arc_get_value(ui_arcTempSettings);
  floor_setpoint = val;
  heater->setTargetTemperature(floor_setpoint);
  nvs.begin("stored_values", RW_MODE);
  nvs.putInt("saved_temp", floor_setpoint);
  nvs.end();
}

static void event_moisture_arc_change (lv_event_t * e) {
  int val = lv_arc_get_value(ui_ArcMoistLevel);
  humidityControl.setTargetHumidity(val);
}

static void event_pump_power_change (lv_event_t * e) {
  int val = lv_slider_get_value(ui_SliderPumpPower);
  humidityControl.setPumpPower(val);
}

static void event_pump_duration_change (lv_event_t * e) {
  int val = lv_slider_get_value(ui_SliderMoistureVolume);
  humidityControl.setPumpDuration(val);
}

static void event_moist_switch_change (lv_event_t * e) {
  bool val = lv_obj_has_state(ui_SwitchMoistOnOff,LV_STATE_CHECKED);
  humidityControl.enableControl(val);
}

void initEventSetup () {
  lv_obj_add_event_cb(ui_ArcMoistLevel, event_moisture_arc_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_SliderPumpPower, event_pump_power_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_SliderMoistureVolume, event_pump_duration_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_SwitchMoistOnOff, event_moist_switch_change, LV_EVENT_VALUE_CHANGED, NULL);
}

/* Display flushing */
void my_disp_flush (lv_display_t *disp, const lv_area_t *area, uint8_t *pixelmap)
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    if (LV_COLOR_16_SWAP) {
        size_t len = lv_area_get_size( area );
        lv_draw_sw_rgb565_swap( pixelmap, len );
    }

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( (uint16_t*) pixelmap, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read (lv_indev_t * indev_driver, lv_indev_data_t * data)
{
    int touchX = 0, touchY = 0;

  if (mytouch.available()) {
    touchX = mytouch.data.x;
    touchY = mytouch.data.y;
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchX;
    data->point.y = touchY;
  } else {
    data->state = LV_INDEV_STATE_REL;
   }
}

void setup ()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    wifiManager.begin();
    wifiManager.connect("RT-GPON-2C0C", "uT7FQQ4K");
    initNvs();
    temp.begin();
    lv_init();

#if LV_USE_LOG != 0
    lv_log_register_print_cb( my_print ); /* register print function for debugging */
#endif

    tft.begin();          /* TFT init */
    tft.setRotation( 0 ); /* Landscape orientation, flipped */

    mytouch.begin();

    static lv_disp_t* disp;
    disp = lv_display_create( screenWidth, screenHeight );
    lv_display_set_buffers( disp, buf, NULL, SCREENBUFFER_SIZE_PIXELS * sizeof(lv_color_t), LV_DISPLAY_RENDER_MODE_PARTIAL );
    lv_display_set_flush_cb( disp, my_disp_flush );

    static lv_indev_t* indev;
    indev = lv_indev_create();
    lv_indev_set_type( indev, LV_INDEV_TYPE_POINTER );
    lv_indev_set_read_cb( indev, my_touchpad_read );

    lv_tick_set_cb( my_tick_get_cb );

    ui_init();
    heater = new HeaterController(TEMP_PIN, RELAY_PIN, "pid");
    heater->setPIDTunings(180.0, 72.0, 112.0);
    heater->setTargetTemperature(floor_setpoint);
    humidityControl.begin();
    initEventSetup();
    lv_arc_set_value(ui_arcTempSettings,floor_setpoint);
    char buf[10];
    dtostrf(floor_setpoint,4,2,buf);
    lv_label_set_text(ui_tempLabel, buf);
    Serial.println( "Setup done" );
}

void loop ()
{
    xSemaphoreTake(gui_mutex,portMAX_DELAY);
    lv_timer_handler(); /* let the GUI do its work */
    xSemaphoreGive(gui_mutex);
    vTaskDelay(5 /portTICK_PERIOD_MS);
}