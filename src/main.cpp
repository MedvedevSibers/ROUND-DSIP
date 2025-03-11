#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <CST816S.h>
#include <QuickPID.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Preferences.h>
#include "wireless_control.cpp"

#define RW_MODE false  //Варианты работы с памятью
#define RO_MODE true

static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

AsyncWiFiManager wifiManager;

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */
CST816S mytouch(22,21,27,14); // пины для работы с тачскрином

#define PUMP_PIN 17
#define TEMP_PIN 32
#define RELAY_PIN 33 // WAS 33

OneWire oneWire(TEMP_PIN);
DallasTemperature temp(&oneWire);

Preferences nvs;

TaskHandle_t TaskTempCheck_t;
SemaphoreHandle_t gui_mutex;

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
    ledcSetup(0,2500,8);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
}

void uiSetValues () {

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

void taskTempCheck (void *pvParameters) {
    float tempVal;
    char tempStr[10];
    while(1){
        temp.requestTemperatures();
        tempVal = temp.getTempCByIndex(0);
        floor_temp = tempVal;
        if (floor_temp >= floor_setpoint + 1){
            digitalWrite(RELAY_PIN, HIGH);
            Serial.print("OFF");
            Serial.print(digitalRead(RELAY_PIN));
        }
        else if (floor_temp < floor_setpoint){
            digitalWrite(RELAY_PIN, LOW);
            Serial.print("ON");
            Serial.print(digitalRead(RELAY_PIN));
        }
        dtostrf(tempVal,7,2,tempStr);
        xSemaphoreTake(gui_mutex, portMAX_DELAY);
        lv_label_set_text(ui_labelActTemp,tempStr);
        xSemaphoreGive(gui_mutex);
        vTaskDelay(300/ portTICK_PERIOD_MS);
    }
}

static void event_arc_temp_change (lv_event_t * e) {
  int val = lv_arc_get_value(ui_arcTempSettings);
  floor_setpoint = val;
  nvs.begin("stored_values", RW_MODE);
  nvs.putInt("saved_temp", floor_setpoint);
  nvs.end();
}

static void event_pump_freq_change (lv_event_t * e) {
  char val[10];
  lv_dropdown_get_selected_str(ui_dropdownPwmFreq, val, sizeof(val));
  ledcChangeFrequency(0, atoi(val),8);
  Serial.printf("PUMP FREQ = ", val);
}

static void event_pump_power_change (lv_event_t * e) {
  int val = lv_arc_get_value(ui_ArcPUMP);
  ledcWrite(0, val);
  Serial.printf("PUMP POWER = ", val);
}

static void event_pump_onoff (lv_event_t * e) {
  if (lv_obj_has_state(ui_switchOnOffPump, LV_STATE_CHECKED)) {
    ledcAttachPin(PUMP_PIN, 0);
    ledcWrite(0, lv_arc_get_value(ui_ArcPUMP));
  }
  else {
    ledcWrite(0,0);
    ledcDetachPin(PUMP_PIN);
  }
}

void initEventSetup () {
  lv_obj_add_event_cb(ui_dropdownPwmFreq, event_pump_freq_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_ArcPUMP, event_pump_power_change, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_switchOnOffPump, event_pump_onoff, LV_EVENT_VALUE_CHANGED, NULL);
  lv_obj_add_event_cb(ui_arcTempSettings, event_arc_temp_change, LV_EVENT_VALUE_CHANGED, NULL);
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
  } else {
    data->state = LV_INDEV_STATE_REL;
   }

  data->point.x = touchX;
  data->point.y = touchY;
}

void setup ()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */
    gui_mutex = xSemaphoreCreateMutex();
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
    initPwmSetup();
    initEventSetup();
    xTaskCreatePinnedToCore(taskTempCheck, "TempTask", 4096, NULL, 1, &TaskTempCheck_t, 0);

    Serial.println( "Setup done" );
}

void loop ()
{
    xSemaphoreTake(gui_mutex,portMAX_DELAY);
    lv_timer_handler(); /* let the GUI do its work */
    xSemaphoreGive(gui_mutex);
    vTaskDelay(5 /portTICK_PERIOD_MS);
}