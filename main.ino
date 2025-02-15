//V 1.0

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
//        8   - SDA
//        9   - SDL


#define LED_PIN    5
#define LED_COUNT 150
#define BRIGHTNESS 50  //(max = 255)
#define STANDBY_LED_TIMER 5000

#define DISTANCE_TRIGGER 200

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
  if(millis()-VL53ReadInterval > 100){
    read_dual_sensors();
    VL53ReadInterval = millis();
  }

  // Update the State Machine
  if(millis()-updateInterval > 300){
    updateTask();
    updateInterval = millis();
  }  
}





void updateTask(){
  bool debug = true;

  Serial.print("State: ");
    Serial.println(state);
    Serial.print("Sensor1: ");
    Serial.print(sensor1Data);
    Serial.print(" mm, Sensor2: ");
    Serial.println(sensor2Data);


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
                colorWipe(strip.Color(255, 0, 0), 20); // Red
                ledTimerStart = millis(); // Start Timer
                state = TIMER_LED_ON;
                break;

            case SENSOR2_TRIGGERED:
                colorWipe(strip.Color(0, 0, 255), 20); // Blue
                ledTimerStart = millis(); // Start Timer
                state = TIMER_LED_ON;
                break;

            case TIMER_LED_ON:
                // Reset Timer if triggered again
                if (sensor1Data < DISTANCE_TRIGGER || sensor2Data < DISTANCE_TRIGGER) {
                    ledTimerStart = millis();
                }
                // Turn off LEDs after 20 seconds
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


void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    delay(wait);                           //  Pause for a moment
  }
}

void fadeOut(int speed) {
    for (int i = 255; i >= 0; i -= speed) {
        strip.setBrightness(i);
        strip.show();
        delay(50);
    }
    strip.clear();
    strip.show();
}
