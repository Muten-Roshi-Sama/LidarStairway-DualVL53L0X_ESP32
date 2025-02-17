//V 1.2

// This project uses 2 ToF VL53L0x Laser Ranging Sensors, a ESP32-C3, a 5m WS2812B Addressable LED Strip and an USB-C Power Supply.



#include <Adafruit_NeoPixel.h>
#include "Adafruit_VL53L0X.h"
#include "dual_VL53LoX.h"


// Pinout :
  //        3V3 - Vin VL53LoX
  //        5V  - Vin LedStrip
  //        
  //        5   - data LedStrip
  //        6   - shutDownLox1
  //        7   - shutDownLox2
  //        8   - SDA           // always same pins 8,9 for I2C on esp32 C3
  //        9   - SDL           // both sensor are connected here

// TODO :
  //      - add ESPHome to control
  //      - only illuminate at night
  //      - fix delay when lighting up the LEDS ?
  //      -


// LED --------
#define LED_PIN    5
#define LED_COUNT 150
#define BRIGHTNESS 50  //(max = 255)
#define STANDBY_LED_TIMER 5000
#define LED_SPEED 20        // speed at which the leds light up one-by-one (ms)

// ToF Time of Flight Laser Ranging Sensor-------
#define DISTANCE_TRIGGER 200
#define ToF_READ_INTERVAL 100

// Update ----
#define UPDATE_TASK_INTERVAL 300


//---------LedStrip----------
#define COLOR_RED      strip.Color(255, 0, 0)
#define COLOR_GREEN    strip.Color(0, 255, 0)
#define COLOR_BLUE     strip.Color(0, 0, 255)
#define COLOR_WHITE    strip.Color(255, 255, 255)

#define COLOR_WARM_WHITE    strip.Color(255, 240, 180)
#define COLOR_RED_WHITE    strip.Color(220, 220, 180)
#define COLOR_COLD_WHITE    strip.Color(255, 255, 220)

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);  // --- WS2812B is GRB


//---------Millis()----------
unsigned long VL53ReadInterval = 0;
unsigned long ledTimerStart = 0;
unsigned long updateInterval = 0;


// External Sensor Data (from dual_VL53LoX.h)
extern int sensor1Data;
extern int sensor2Data;

enum State { INIT, IDLE, SENSOR1_TRIGGERED, SENSOR2_TRIGGERED, TIMER_LED_ON, EXIT };
State state = INIT;

// ===================SETUP FUNCTION==============================
void setup() {
  Serial.begin(115200);
  while (! Serial) { delay(1); }  // wait until serial port opens for native USB devices

  // Initialize Components
  init_VL53Lox();
  initLed();
}


void loop() {
  // Read Sensors
  if(millis()-VL53ReadInterval > ToF_READ_INTERVAL){
    read_dual_sensors();
    VL53ReadInterval = millis();
  }

  // Update the State Machine
  if(millis()-updateInterval > UPDATE_TASK_INTERVAL){
    updateTask();
    updateInterval = millis();
  }  

  // Serial.print("State: ", state, ", Sensor1: ", mea)


}





void updateTask(){
  bool debug = true;

  if(debug){
     Serial.print("State: ");
      Serial.print(state);
      Serial.print(", Sensor1: ");
      Serial.print(sensor1Data);
      Serial.print(" mm, Sensor2: ");
      Serial.print(sensor2Data);
      Serial.println(" mm");
  }
   

  switch (state) {
            case INIT:
                strip.clear();
                strip.show();
                state = IDLE;
                break;

            case IDLE:
                strip.clear();
                strip.show();
                if (sensor1Data < DISTANCE_TRIGGER) {
                    state = SENSOR1_TRIGGERED;
                } else if (sensor2Data < DISTANCE_TRIGGER) {
                    state = SENSOR2_TRIGGERED;
                }
                break;

            case SENSOR1_TRIGGERED:
                lightUpDirection(COLOR_WARM_WHITE, LED_SPEED, true);
                ledTimerStart = millis(); // Start Timer
                state = TIMER_LED_ON;
                break;

            case SENSOR2_TRIGGERED:
                lightUpDirection(COLOR_COLD_WHITE, LED_SPEED, false);
                ledTimerStart = millis(); // Start Timer
                state = TIMER_LED_ON;
                break;

            case TIMER_LED_ON:
                // Reset Timer if triggered again
                if (sensor1Data < DISTANCE_TRIGGER || sensor2Data < DISTANCE_TRIGGER) {
                    ledTimerStart = millis();
                }
                // Turn off LEDs after delay
                if (millis() - ledTimerStart > STANDBY_LED_TIMER) {
                    state = EXIT;
                }
                break;

            case EXIT:
                fadeOut(20);
                state = IDLE;
                break;
        }
}



void initLed(){
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(BRIGHTNESS); // Set BRIGHTNESS to about 1/5 (max = 255)
}



// =================================================
//              LED COLORS & DRAWING
// =================================================
void lightUpDirection(uint32_t color, int wait, bool topToBottom) {
  if (topToBottom) {
    for (int i = 0; i < strip.numPixels(); i++) { // Top to bottom
      strip.setPixelColor(i, color);
      strip.show();
      delay(wait);
    }
  } else {
    for (int i = strip.numPixels() - 1; i >= 0; i--) { // Bottom to top
      strip.setPixelColor(i, color);
      strip.show();
      delay(wait);
    }
  }
}

void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void fadeOut(int speed) {
    for (int i = BRIGHTNESS; i >= 0; i -= speed) {
        strip.setBrightness(i);
        strip.show();
        delay(100);
    }
    strip.clear();
    strip.show();
}
