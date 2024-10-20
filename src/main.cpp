//Arduino-TFT_eSPI board-template main routine. There's a TFT_eSPI create+flush driver already in LVGL-9.1 but we create our own here for more control (like e.g. 16-bit color swap).

#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>
#include <CST816S.h>

/*Don't forget to set Sketchbook location in File/Preferences to the path of your UI project (the parent foder of this INO file)*/

/*Change to your screen resolution*/
static const uint16_t screenWidth  = 240;
static const uint16_t screenHeight = 240;

enum { SCREENBUFFER_SIZE_PIXELS = screenWidth * screenHeight / 10 };
static lv_color_t buf [SCREENBUFFER_SIZE_PIXELS];

TFT_eSPI tft = TFT_eSPI( screenWidth, screenHeight ); /* TFT instance */
CST816S mytouch(22,21,27,14);

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char * buf)
{
    Serial.printf(buf);
    Serial.flush();
}
#endif

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

  /*Set the coordinates*/
  data->point.x = touchX;
  data->point.y = touchY;

//I added these conditions just to keep my serial monitor from going crazy
  if (touchX != 0) {
    Serial.print("Data x ");
    Serial.println(touchX);
    Serial.print(mytouch.data.version);
  }

  if (touchY != 0) {
    Serial.print("Data y ");
    Serial.println(touchY);
    Serial.print(mytouch.gesture());
  }
}
/*Set tick routine needed for LVGL internal timings*/
static uint32_t my_tick_get_cb (void) { return millis(); }




void setup ()
{
    Serial.begin( 115200 ); /* prepare for possible serial debug */

    String LVGL_Arduino = "Hello Arduino! ";
    LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

    Serial.println( LVGL_Arduino );
    Serial.println( "I am LVGL_Arduino" );

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

    Serial.println( "Setup done" );
}

void loop ()
{
    lv_timer_handler(); /* let the GUI do its work */
    delay(5);
}







// #include <Wire.h>
// #include <CST816_TouchLib.h>

// #define PIN_TFT_POWER_ON	21
// #define I2C_SDA				15
// #define I2C_SCL				16

// using namespace MDO;

// CST816Touch oTouch;

// void setup () {
// 	Serial.begin(115200);

// 	pinMode(PIN_TFT_POWER_ON, OUTPUT);				//TFT poweron
// 	digitalWrite(PIN_TFT_POWER_ON, HIGH);	

// 	Wire.begin(I2C_SDA, I2C_SCL);
// 	Wire.setClock(400000);	//For reliable communication, it is recommended to use a *maximum* communication rate of 400Kbps

// 	if (!oTouch.begin(Wire)) {
// 		Serial.println("Touch screen initialization failed..");
// 		while(true){
// 			delay(100);
// 		}
// 	}

// 	if (!oTouch.setOperatingModeHardwareBased()) {
// 		Serial.println("Set full hardware operational mode failed");
// 	}

// //	if (oTouch.setNotifyOnMovement()) {
// //		oTouch.setMovementInterval(100);	//as example: limit to 10 per second (so 100 msec as interval)
// //	} else {
// //		//only available in 'fast'-mode
// //		Serial.println("Set notify on movement failed");
// //	}
			
// 	CST816Touch::device_type_t eDeviceType;
// 	if (oTouch.getDeviceType(eDeviceType)) {
// 		Serial.print("Device is of type: ");
// 		Serial.println(CST816Touch::deviceTypeToString(eDeviceType));
// 	}
		
// 	Serial.println("Touch screen initialization done");
// }


// void loop () {
// 	oTouch.control();
	
// 	if (oTouch.hadTouch()) {
// 		int x = 0;
// 		int y = 0;
// 		oTouch.getLastTouchPosition(x, y);
		
// 		Serial.print("Touch received at: (");
// 		Serial.print(x);
// 		Serial.print(",");
// 		Serial.print(y);
// 		Serial.println(")");
// 	}
	
// 	if (oTouch.hadGesture()) {	//note that a gesture typically starts with a touch. Both will be provided here.
// 		CST816Touch::gesture_t eGesture;
// 		int x = 0;
// 		int y = 0;
// 		oTouch.getLastGesture(eGesture, x, y);
		
// 		Serial.print("Gesture (");
// 		Serial.print(CST816Touch::gestureIdToString(eGesture));
// 		Serial.print(") received at: (");
// 		Serial.print(x);
// 		Serial.print(",");
// 		Serial.print(y);
// 		Serial.println(")");
// 	}	
// }

// #include <Arduino.h>
// #include <Wire.h>
 
// void setup(){
//     Wire.begin(22,21);    
 
//     Serial.begin(115200);
//     while (!Serial);
//     Serial.println("\nI2C Scanner");
// } 
 
// void loop(){
//     byte error, address;
//     int nDevices;
 
//     Serial.println("Scanning...");
 
//     nDevices = 0;
//     for(address = 8; address < 127; address++ ){
//         Wire.beginTransmission(address);
//         error = Wire.endTransmission();
 
//         if (error == 0){
//             Serial.print("I2C device found at address 0x");
//             if (address<16)
//                 Serial.print("0");
//             Serial.print(address,HEX);
//             Serial.println(" !");
 
//             nDevices++;
//         }
//         else if (error==4) {
//             Serial.print("Unknow error at address 0x");
//             if (address<16)
//                 Serial.print("0");
//             Serial.println(address,HEX);
//         } 
//     }
//     if (nDevices == 0)
//         Serial.println("No I2C devices found\n");
//     else
//         Serial.println("done\n");
 
//     delay(5000);
// }

// #include <Wire.h>
// #include <Arduino.h>

// // Define AHT10 register addresses
// #define AHT10_ADDR 0x38
// #define CMD_MEASURE 0xAC
// #define CMD_SOFTRESET 0xBA
// #define STATUS_BUSY_MASK 0x80

// // Define I2C0 pins
// #define SDA_PIN 21
// #define SCL_PIN 22

// // Global variables for sensor data
// float temperature = 0.0;
// float humidity = 0.0;

// // Function to read AHT10 sensor
// void readAHT10() {
//   // Send measurement command to AHT10
//   Wire.beginTransmission(AHT10_ADDR);
//   Wire.write(CMD_MEASURE);
//   Wire.write(0x33);
//   Wire.write(0x00);
//   Wire.endTransmission();
  
//   // Wait for measurement to complete
//   while (true) {
//     Wire.beginTransmission(AHT10_ADDR);
//     Wire.write(0x71);
//     Wire.endTransmission(false);
//     Wire.requestFrom(AHT10_ADDR, 1);
//     if ((Wire.read() & STATUS_BUSY_MASK) == 0) break;
//     delay(10);
//   }

//   // Read measurement data from AHT10
//   Wire.beginTransmission(AHT10_ADDR);
//   Wire.write(0x00);
//   Wire.endTransmission(false);
//   Wire.requestFrom(AHT10_ADDR, 6);
//   uint8_t msb = Wire.read();
//   uint8_t lsb = Wire.read();
//   uint8_t cksum = Wire.read();
//   uint32_t raw_data = ((msb << 16) | (lsb << 8) | cksum) >> 4;

//   // Convert raw data to temperature and humidity
//   temperature = (float)(raw_data * 200.0 / 1048576.0) - 50.0;
//   humidity = (float)(raw_data * 100.0 / 1048576.0);
// }

// void setup() {
//   // Initialize serial communication
//   Serial.begin(115200);

//   // Initialize I2C communication on I2C0 with custom SDA and SCL pins
//   Wire.begin(SDA_PIN, SCL_PIN);
  
//   // Soft reset AHT10
//   Wire.beginTransmission(AHT10_ADDR);
//   Wire.write(CMD_SOFTRESET);
//   Wire.endTransmission();
//   delay(20);
// }

// void loop() {
//   // Read data from AHT10
//   readAHT10();

//   // Print temperature and humidity values to serial monitor
//   Serial.print("Temperature: ");
//   Serial.print(temperature, 2); // round to 2 decimal places
//   Serial.print(" C, Humidity: ");
//   Serial.print(humidity, 2); // round to 2 decimal places
//   Serial.println(" %");

//   // Wait 1 second before taking another measurement
//   delay(1000);
// }